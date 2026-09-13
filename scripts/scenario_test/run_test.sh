#!/bin/bash
# シナリオテストの実行スクリプト

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
VENV_DIR="${REPO_ROOT}/scenario_test_env"

# 引数の解析
TEST_NAME="${1:-all}"
VISION_PORT="${VISION_PORT:-10020}"
USE_LOCAL="${USE_LOCAL:-1}" # デフォルトはローカルモード
CRANE_TAG="${CRANE_TAG:-local-scenario}"
PLANNER="${PLANNER:-rvo2}"

# CM4 in the loop 構成用の設定。
# COMPOSE_PROFILES=cm4-loop を指定すると cm4-sim が経路に入る。
# そのときは crane の送信先を cm4-sim(12345) に向け、simulator-cli を 12346 へ退避させ、
# feedback は実機と同じ multicast で受ける（FEEDBACK_SIM_MODE=false）必要がある。
# FEEDBACK_SIM_MODE=true のままだと crane と cm4-sim が 127.0.0.1:50100+id を奪い合い、
# 片方が全パケットを取るため cm4-sim の位置制御ループが位置信号を失う。
COMPOSE_PROFILES="${COMPOSE_PROFILES:-}"
IBIS_PORT="${IBIS_PORT:-12345}"
# simulator-cli の ibis チーム色。crane の team 引数と揃えること。
# referee からの自動検出(--ibis-use-referee)は rcst/autoref 環境では
# チーム名 ibis を引けず永久に解決せず、feedback が 1 パケットも出ない。
IBIS_TEAM_COLOR="${IBIS_TEAM_COLOR:-yellow}"
CRANE_TARGET_PORT="${CRANE_TARGET_PORT:-12345}"
FEEDBACK_SIM_MODE="${FEEDBACK_SIM_MODE:-true}"
RX_DELAY_MS="${RX_DELAY_MS:-0}"
RX_JITTER_MS="${RX_JITTER_MS:-0}"
RX_LOSS_RATE="${RX_LOSS_RATE:-0.0}"

# ローカルモードで cm4-loop profile を使うときのホスト側 cm4_sim バイナリ。
# Orion_CM4 にまだ Dockerfile が無く ghcr.io/ibis-ssl/orion-cm4-sim も未作成のため、
# ローカルモードでは crane と同じくホスト上で直接起動する。
CM4_SIM_BIN="${CM4_SIM_BIN:-${REPO_ROOT}/../../../Orion_CM4/cm4/bin/cm4_sim.out}"
CM4_ROBOT_IDS="${CM4_ROBOT_IDS:-0,1,2,3,4,5,6,7,8,9,10}"
CM4_RATE_HZ="${CM4_RATE_HZ:-1000}"
CM4_SEED="${CM4_SEED:-0}"

# cm4-loop profile が指定されているか
USE_CM4_LOOP=0
case ",${COMPOSE_PROFILES}," in
*,cm4-loop,*) USE_CM4_LOOP=1 ;;
esac

# compose に渡す profile 引数を組み立てる（カンマ区切りで複数指定可）
COMPOSE_PROFILE_ARGS=()
if [ -n "${COMPOSE_PROFILES}" ]; then
    IFS=',' read -r -a _profiles <<<"${COMPOSE_PROFILES}"
    for _p in "${_profiles[@]}"; do
        [ -n "${_p}" ] && COMPOSE_PROFILE_ARGS+=(--profile "${_p}")
    done
fi

# ワークスペースルートのパス（REPO_ROOTの2階層上）
WORKSPACE_ROOT="$(cd "${REPO_ROOT}/../.." && pwd)"

# Docker Compose設定（ローカルモードとリモートモードで切り替え）
if [ "${USE_LOCAL}" = "1" ]; then
    COMPOSE_FILE="${REPO_ROOT}/docker/scenario/docker-compose.local.yaml"
    MODE_NAME="ローカル（マウント）"
    # ローカルモード用にワークスペースルートを環境変数で設定
    export IBIS_WS="${WORKSPACE_ROOT}"
else
    COMPOSE_FILE="${REPO_ROOT}/docker/scenario/docker-compose.yaml"
    MODE_NAME="リモート（イメージ）"
fi

# ログレコーダーのパス
LOG_RECORDER="${REPO_ROOT}/ssl-log-recorder"

echo "=== シナリオテストの実行 ==="
echo "モード: ${MODE_NAME}"
echo "テスト: ${TEST_NAME}"
echo "プランナー: ${PLANNER}"
if [ "${USE_LOCAL}" != "1" ]; then
    echo "Dockerイメージタグ: ${CRANE_TAG}"
fi
echo ""

# Python仮想環境の確認
if [ ! -d "${VENV_DIR}" ]; then
    echo "エラー: Python仮想環境が見つかりません"
    echo "先に 'make scenario-test-setup' を実行してください"
    exit 1
fi

# 仮想環境の有効化
# shellcheck source=/dev/null
source "${VENV_DIR}/bin/activate"

# ssl-log-recorderをダウンロード（ログ記録用）
if [ ! -f "${LOG_RECORDER}" ]; then
    echo "ssl-log-recorderをダウンロード中..."
    curl -L https://github.com/RoboCup-SSL/ssl-go-tools/releases/download/v1.5.2/ssl-log-recorder_v1.5.2_linux_amd64 -o "${LOG_RECORDER}"
    chmod +x "${LOG_RECORDER}"
    echo "ダウンロード完了"
fi

# 念のため前回の残存コンテナを停止・削除。
# ここは「前回の実行」の残骸が対象で、前回が別 profile だった可能性があるため、
# 常に cm4-loop を有効にして down する（profile を外すと cm4-sim が削除対象から漏れ、
# 12345 を bind したまま残って今回の構成と衝突する）。
docker compose --profile cm4-loop -f "${COMPOSE_FILE}" down 2>/dev/null || true

# erforce-sim/auto-refereeはnetwork_mode:hostでマルチキャストを使うため、
# Wi-Fi/LANへの漏洩を防ぐホスト隔離設定を適用してから起動する
"${REPO_ROOT}/scripts/ensure-sim-network-confined.sh"

# Docker Composeでサービスを起動（grSimとauto-referee）
echo "Docker Composeでサービスを起動中..."
cd "${REPO_ROOT}"
CRANE_TAG="${CRANE_TAG}" PLANNER="${PLANNER}" \
    IBIS_PORT="${IBIS_PORT}" IBIS_TEAM_COLOR="${IBIS_TEAM_COLOR}" CRANE_TARGET_PORT="${CRANE_TARGET_PORT}" \
    FEEDBACK_SIM_MODE="${FEEDBACK_SIM_MODE}" \
    RX_DELAY_MS="${RX_DELAY_MS}" RX_JITTER_MS="${RX_JITTER_MS}" RX_LOSS_RATE="${RX_LOSS_RATE}" \
    docker compose "${COMPOSE_PROFILE_ARGS[@]}" -f "${COMPOSE_FILE}" up -d

# ローカルモードかつ cm4-loop の場合、cm4_sim をホスト上で起動する。
# crane より先に上げて 12345 を確保しておく（crane の送信先）。
CM4_SIM_PID=""
if [ "${USE_LOCAL}" = "1" ] && [ "${USE_CM4_LOOP}" = "1" ]; then
    if [ ! -x "${CM4_SIM_BIN}" ]; then
        echo "エラー: cm4_sim バイナリが見つかりません: ${CM4_SIM_BIN}"
        echo "Orion_CM4 の feat/cm4-position-control で 'bash cm4/build.sh' を実行するか、"
        echo "CM4_SIM_BIN で場所を指定してください"
        exit 1
    fi
    echo "cm4_sim をホスト上で起動中..."
    # --feedback-port-base は 50100 固定。cm4_sim の再配信先は
    # 224.5.20.(100+id):<base>+id で base に連動するが、crane_robot_receiver 側は
    # 50100 を直書きしているため、ずらすと feedback が無言で途切れる。
    "${CM4_SIM_BIN}" \
        --robot-ids "${CM4_ROBOT_IDS}" \
        --in-port "${CRANE_TARGET_PORT}" \
        --out-addr 127.0.0.1 \
        --out-port "${IBIS_PORT}" \
        --feedback-port-base 50100 \
        --multicast-if 127.0.0.1 \
        --rate-hz "${CM4_RATE_HZ}" \
        --rx-delay-ms "${RX_DELAY_MS}" \
        --rx-jitter-ms "${RX_JITTER_MS}" \
        --rx-loss-rate "${RX_LOSS_RATE}" \
        --seed "${CM4_SEED}" >/tmp/cm4_sim_local.log 2>&1 &
    CM4_SIM_PID=$!
    echo "cm4_simプロセスID: ${CM4_SIM_PID}"
fi

# ローカルモードの場合、craneをローカルで起動
CRANE_PID=""
if [ "${USE_LOCAL}" = "1" ]; then
    echo "ローカル環境でcraneを起動中..."
    cd "${WORKSPACE_ROOT}"

    # ROS 2環境のセットアップとcraneの起動（バックグラウンド）
    # shellcheck source=/dev/null
    source "${WORKSPACE_ROOT}/install/setup.bash"
    # setsid で独自のプロセスグループにする。非対話スクリプトはジョブ制御が無効なので、
    # 単に & で起動するとこのスクリプト自身と同じプロセスグループに入ってしまい、
    # 後段の `kill -TERM -<pid>` が存在しないグループを狙って何も落とせない。
    # その結果 ros2 launch の子ノード群が毎回残り、実行を繰り返すと DDS domain 0 の
    # participant index を使い切って world_model_publisher が起動できなくなる。
    setsid ros2 launch crane_bringup crane.launch.xml sim:=true speak:=false vision_port:=10020 referee_port:=10003 team:=Yellow planner:="${PLANNER}" ibis_target_port:="${CRANE_TARGET_PORT}" feedback_sim_mode:="${FEEDBACK_SIM_MODE}" >/tmp/crane_local.log 2>&1 &
    CRANE_PID=$!
    echo "craneプロセスID: ${CRANE_PID}"

    cd "${REPO_ROOT}"
fi

# サービスの起動を待機
echo "サービスの起動を待機中..."
sleep 5

# テスト実行
echo ""
echo "=== テストを実行中 ==="

cd "${REPO_ROOT}"

# pytestコマンドの構築（常にログ記録を有効化）
PYTEST_ARGS=(
    "--vision_port=${VISION_PORT}"
    "--logging"
    "--log_recorder=${LOG_RECORDER}"
    "-p" "no:launch_ros"
    "-p" "no:launch_testing"
    "-p" "no:launch_pytest"
)

# テストの実行（失敗してもスクリプトは継続）
set +e # 一時的にエラーで終了しないようにする
if [ "${TEST_NAME}" = "all" ]; then
    echo "全テストを実行します（ログ記録有効）..."
    pytest scenario_test/ "${PYTEST_ARGS[@]}"
else
    echo "テスト ${TEST_NAME} を実行します（ログ記録有効）..."
    pytest "scenario_test/${TEST_NAME}.py" "${PYTEST_ARGS[@]}"
fi

TEST_RESULT=$?
set -e # エラー終了を再び有効化

# ログの表示
echo ""
echo "=== Dockerコンテナのログ ==="
docker compose "${COMPOSE_PROFILE_ARGS[@]}" -f "${COMPOSE_FILE}" logs

# 動画生成（テスト失敗時のみ、最新のログファイル1件のみ対象）
if [ ${TEST_RESULT} -ne 0 ]; then
    echo ""
    echo "=== テスト失敗：ログから動画を生成中 ==="

    # ssl-log-videoのセットアップ
    SSL_LOG_VIDEO_DIR="${REPO_ROOT}/ssl-go-tools"
    if [ ! -d "${SSL_LOG_VIDEO_DIR}" ]; then
        echo "ssl-go-toolsをクローン中..."
        git clone https://github.com/ibis-ssl/ssl-go-tools.git -b video "${SSL_LOG_VIDEO_DIR}"
    fi

    # ssl-log-videoのビルド
    cd "${SSL_LOG_VIDEO_DIR}/cmd/ssl-log-video"
    if [ ! -f "ssl-log-video" ]; then
        echo "ssl-log-videoをビルド中..."
        go mod tidy
        go build -o ssl-log-video ssl-log-video.go
        chmod +x ssl-log-video
    fi

    # ffmpegのインストール確認
    if ! command -v ffmpeg &>/dev/null; then
        echo "ffmpegをインストール中..."
        sudo apt update
        sudo apt install -y ffmpeg
    fi

    # 直近に生成された最新ログのみ動画変換
    cd "${REPO_ROOT}"
    # shellcheck disable=SC2012,SC2035
    LATEST_LOG="$(ls -t ./*.log.gz 2>/dev/null | head -n 1)"
    if [ -n "${LATEST_LOG}" ] && [ -f "${LATEST_LOG}" ]; then
        echo "動画を生成中: ${LATEST_LOG}"
        gunzip -c "${LATEST_LOG}" >"${LATEST_LOG%.gz}"
        "${SSL_LOG_VIDEO_DIR}/cmd/ssl-log-video/ssl-log-video" -file "${LATEST_LOG%.gz}" -output "${LATEST_LOG%.gz}.avi" || true
        ffmpeg -i "${LATEST_LOG%.gz}.avi" -vcodec libx264 -acodec aac "${LATEST_LOG%.gz}.mp4" -y || true
        echo "動画を生成しました: ${LATEST_LOG%.gz}.mp4"
    fi
fi

# クリーンアップ
echo ""

# ローカルモードの場合、craneプロセスを停止
if [ -n "${CRANE_PID}" ]; then
    echo "=== ローカルcraneプロセスを停止中 ==="
    # プロセスグループ全体を終了（子プロセスも含む）。
    # pid ではなく実際の pgid を引く。setsid が効いていれば pgid == CRANE_PID になるが、
    # 万一効いていない場合にこのスクリプト自身のグループを撃たないよう $$ と比較して守る。
    CRANE_PGID="$(ps -o pgid= -p "${CRANE_PID}" 2>/dev/null | tr -d ' ')"
    if [ -n "${CRANE_PGID}" ] && [ "${CRANE_PGID}" != "$$" ]; then
        kill -TERM -"${CRANE_PGID}" 2>/dev/null || true
        sleep 2
        kill -KILL -"${CRANE_PGID}" 2>/dev/null || true
    else
        echo "警告: craneのプロセスグループを特定できないため単一プロセスのみ停止します" >&2
        kill -TERM "${CRANE_PID}" 2>/dev/null || true
        sleep 2
        kill -KILL "${CRANE_PID}" 2>/dev/null || true
    fi
    echo "craneプロセスを停止しました"

    # craneのログを表示
    if [ -f "/tmp/crane_local.log" ]; then
        echo ""
        echo "=== ローカルcraneのログ ==="
        tail -50 /tmp/crane_local.log
    fi
fi

# cm4_simプロセスを停止
if [ -n "${CM4_SIM_PID}" ]; then
    echo "=== ローカルcm4_simプロセスを停止中 ==="
    kill -TERM "${CM4_SIM_PID}" 2>/dev/null || true
    sleep 1
    kill -KILL "${CM4_SIM_PID}" 2>/dev/null || true
    echo "cm4_simプロセスを停止しました"
    if [ -f "/tmp/cm4_sim_local.log" ]; then
        echo ""
        echo "=== ローカルcm4_simのログ ==="
        tail -30 /tmp/cm4_sim_local.log
    fi
fi

echo ""
echo "=== Docker Composeサービスを停止中 ==="
docker compose "${COMPOSE_PROFILE_ARGS[@]}" -f "${COMPOSE_FILE}" down

echo ""
if [ ${TEST_RESULT} -eq 0 ]; then
    echo "✅ テストが成功しました"
    echo ""
    echo "生成されたログファイル:"
    ls -lh ./*.log* 2>/dev/null || echo "  (ログファイルなし)"
else
    echo "❌ テストが失敗しました"
    echo ""
    echo "生成されたログファイル:"
    ls -lh ./*.log* 2>/dev/null || echo "  (ログファイルなし)"
    echo ""
    echo "生成された動画ファイル:"
    ls -lh ./*.mp4 2>/dev/null || echo "  (動画ファイルなし)"
fi

exit ${TEST_RESULT}
