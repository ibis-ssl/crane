#!/bin/bash
# 旧構成（Arm A）と新構成（Arm B）を同一の劣化条件で走らせて追従品質を比較する。
#
#   Arm A (baseline): crane(rvo2, wire mode 3) --12345--> simulator-cli
#                     位置制御ループは crane 側
#   Arm B (cm4)     : crane(visibility_graph, wire mode 4) --12345--> cm4-sim --12346--> simulator-cli
#                     位置制御ループは cm4-sim 側
#
# 主張は「無線経路を位置制御ループの外に出すと遅延・ジッタ・ロスに強くなる」。
#
# 【重要な制約 / 未解決】
#   Arm A は crane -> simulator-cli の直送なので、経路上に劣化注入器が存在しない。
#   劣化注入器を持つのは cm4-sim だけである。したがって現状このスクリプトは
#   「Arm B に劣化を入れたときの劣化耐性」しか測れず、厳密な A/B にはなっていない。
#   両アームの注入点を揃えるには次のいずれかが必要（計画で未決）:
#     案1: crane と simulator-cli の間に汎用 UDP 劣化プロキシを挟む
#     案2: cm4-sim に --passthrough（位置制御せず mode 3 を転送し劣化注入だけ行う）を足す
#   Arm A の行は「劣化なしの基準値」として読むこと。
#
#   また Arm A と Arm B では planner が異なる（rvo2 vs visibility_graph）。
#   crane の ibis 経路には位置制御が無いため visibility_graph のまま mode 3 を出せず、
#   この交絡は現状のコードでは解消できない。結果を読む際は考慮すること。
#
# 使い方:
#   bash scripts/scenario_test/run_ab_comparison.sh [出力ディレクトリ]
#
# 環境変数:
#   TRIALS      1 条件あたりの試行回数（既定 10）
#   TIMEOUT     1 試行のタイムアウト秒（既定 20）
#   CONDITIONS  "delay_ms:jitter_ms:loss_rate" の空白区切りリスト
#   ARMS        測定するアーム（既定 "baseline cm4"）
#   USE_LOCAL   1: ローカルワークスペース / 0: Docker イメージ（既定 1）

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
WORKSPACE_ROOT="$(cd "${REPO_ROOT}/../.." && pwd)"
VENV_DIR="${REPO_ROOT}/scenario_test_env"

OUT_DIR="${1:-${REPO_ROOT}/ab_results/$(date +%Y%m%d_%H%M%S)}"
TRIALS="${TRIALS:-10}"
TIMEOUT="${TIMEOUT:-20}"
USE_LOCAL="${USE_LOCAL:-1}"
CRANE_TAG="${CRANE_TAG:-local-scenario}"
ARMS="${ARMS:-baseline cm4}"
# delay_ms:jitter_ms:loss_rate
CONDITIONS="${CONDITIONS:-0:0:0.0 30:0:0.0 60:0:0.0 30:10:0.0 30:10:0.02}"

if [ "${USE_LOCAL}" = "1" ]; then
    COMPOSE_FILE="${REPO_ROOT}/docker/scenario/docker-compose.local.yaml"
    export IBIS_WS="${WORKSPACE_ROOT}"
else
    COMPOSE_FILE="${REPO_ROOT}/docker/scenario/docker-compose.yaml"
fi

if [ ! -d "${VENV_DIR}" ]; then
    echo "エラー: Python環境がセットアップされていません"
    echo "先に 'make scenario-test-setup' を実行してください"
    exit 1
fi

mkdir -p "${OUT_DIR}"
echo "=== A/B 比較 ==="
echo "出力先   : ${OUT_DIR}"
echo "試行回数 : ${TRIALS} / 条件"
echo "アーム   : ${ARMS}"
echo "条件     : ${CONDITIONS}"
echo ""

# shellcheck source=/dev/null
source "${VENV_DIR}/bin/activate"

CRANE_PID=""

# teardown は常に全 profile を有効にして行う。
# profile を外して down すると、その profile のサービス（cm4-sim）が削除対象から漏れる。
# cm4 セルの次に baseline セルを回すと cm4-sim が生き残り、baseline 側でも 12345 を
# bind してしまうため、A/B 比較そのものが汚染される（どちらが受けるかは 4-tuple ハッシュ次第）。
compose_down_all() {
    docker compose --profile cm4-loop -f "${COMPOSE_FILE}" down 2>/dev/null || true
}

cleanup() {
    if [ -n "${CRANE_PID}" ]; then
        kill -TERM -"${CRANE_PID}" 2>/dev/null || true
        sleep 1
        kill -KILL -"${CRANE_PID}" 2>/dev/null || true
        CRANE_PID=""
    fi
    compose_down_all
}
trap cleanup EXIT

run_cell() {
    local arm="$1" delay="$2" jitter="$3" loss="$4"
    local tag="${arm}_d${delay}_j${jitter}_l${loss}"
    local out_json="${OUT_DIR}/${tag}.json"

    echo "----------------------------------------------------------------"
    echo ">>> arm=${arm} delay=${delay}ms jitter=${jitter}ms loss=${loss}"

    if [ "${arm}" = "cm4" ]; then
        PROFILE_ARGS=(--profile cm4-loop)
        local planner="visibility_graph" target_port="12345" ibis_port="12346" feedback_sim="false"
    else
        PROFILE_ARGS=()
        local planner="rvo2" target_port="12345" ibis_port="12345" feedback_sim="true"
    fi

    compose_down_all

    CRANE_TAG="${CRANE_TAG}" PLANNER="${planner}" \
        IBIS_PORT="${ibis_port}" CRANE_TARGET_PORT="${target_port}" \
        FEEDBACK_SIM_MODE="${feedback_sim}" \
        RX_DELAY_MS="${delay}" RX_JITTER_MS="${jitter}" RX_LOSS_RATE="${loss}" \
        docker compose "${PROFILE_ARGS[@]}" -f "${COMPOSE_FILE}" up -d

    if [ "${USE_LOCAL}" = "1" ]; then
        cd "${WORKSPACE_ROOT}"
        # shellcheck source=/dev/null
        source "${WORKSPACE_ROOT}/install/setup.bash"
        ros2 launch crane_bringup crane.launch.xml sim:=true speak:=false \
            vision_port:=10020 referee_port:=10003 team:=Yellow \
            planner:="${planner}" ibis_target_port:="${target_port}" \
            feedback_sim_mode:="${feedback_sim}" \
            >"/tmp/crane_ab_${tag}.log" 2>&1 &
        CRANE_PID=$!
        cd "${REPO_ROOT}"
    fi

    sleep 8 # 起動待ち

    set +e
    cd "${REPO_ROOT}"
    PLANNER="${planner}" CRANE_TARGET_PORT="${target_port}" IBIS_PORT="${ibis_port}" \
        FEEDBACK_SIM_MODE="${feedback_sim}" COMPOSE_PROFILES="${arm}" \
        RX_DELAY_MS="${delay}" RX_JITTER_MS="${jitter}" RX_LOSS_RATE="${loss}" \
        python3 scenario_test/measure_tracking_ab.py \
        --config "${arm}" --trials "${TRIALS}" --timeout "${TIMEOUT}" --out "${out_json}"
    local rc=$?
    set -e

    if [ ${rc} -ne 0 ]; then
        echo "!!! 計測に失敗しました (rc=${rc}): ${tag}"
        if [ "${arm}" = "cm4" ]; then
            echo "    cm4-sim イメージが未実装の可能性があります（Orion_CM4 側で作成中）。"
        fi
    fi

    cleanup
    sleep 2
}

PROFILE_ARGS=()
for arm in ${ARMS}; do
    for cond in ${CONDITIONS}; do
        IFS=':' read -r delay jitter loss <<<"${cond}"
        run_cell "${arm}" "${delay}" "${jitter}" "${loss}"
    done
done

echo ""
echo "=== 比較表 ==="
python3 - "${OUT_DIR}" <<'PYEOF'
import glob, json, os, sys

out_dir = sys.argv[1]
rows = []
for path in sorted(glob.glob(os.path.join(out_dir, "*.json"))):
    if os.path.basename(path) == "comparison.json":
        continue
    try:
        with open(path, encoding="utf-8") as f:
            d = json.load(f)
    except (OSError, ValueError) as e:
        print("読み込み失敗 %s: %s" % (path, e))
        continue
    c, s = d.get("config", {}), d.get("summary", {})
    deg = c.get("degradation", {})
    rows.append({
        "arm": c.get("arm", "?"),
        "delay_ms": deg.get("rx_delay_ms", "?"),
        "jitter_ms": deg.get("rx_jitter_ms", "?"),
        "loss": deg.get("rx_loss_rate", "?"),
        "reach_rate": s.get("reach_rate"),
        "t_goal_mean": s.get("time_to_goal_mean"),
        "dev_mean": s.get("max_lateral_deviation_mean"),
        "overshoot_mean": s.get("overshoot_mean"),
        "path_eff_mean": s.get("path_efficiency_mean"),
    })

def fmt(v):
    if v is None:
        return "-"
    return "%.3f" % v if isinstance(v, float) else str(v)

hdr = ["arm", "delay_ms", "jitter_ms", "loss", "reach_rate", "t_goal_mean", "dev_mean", "overshoot_mean", "path_eff_mean"]
widths = [max(len(h), max((len(fmt(r[h])) for r in rows), default=0)) for h in hdr]
print(" | ".join(h.ljust(w) for h, w in zip(hdr, widths)))
print("-+-".join("-" * w for w in widths))
for r in rows:
    print(" | ".join(fmt(r[h]).ljust(w) for h, w in zip(hdr, widths)))

with open(os.path.join(out_dir, "comparison.json"), "w", encoding="utf-8") as f:
    json.dump(rows, f, indent=2, ensure_ascii=False)
print("\n比較表を %s/comparison.json に保存しました。" % out_dir)
PYEOF

echo ""
echo "個別の結果 JSON: ${OUT_DIR}/"
