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
# mode 4（位置指令）を出すのは visibility_graph だけ。rvo2 は mode 3 を出す。
PLANNER="${PLANNER:-visibility_graph}"

# 標準構成のトポロジ:
#   crane --12345 mode4--> cm4-sim --12346 mode3--> simulator-cli
# 位置制御ループは cm4-sim（実機 CM4 相当）が閉じる。
# CRANE_TARGET_PORT が cm4-sim の入力、IBIS_PORT が simulator-cli の入力。
IBIS_PORT="${IBIS_PORT:-12346}"
# simulator-cli の ibis チーム色。crane の team 引数と揃えること。
# referee からの自動検出(--ibis-use-referee)は rcst/autoref 環境では
# チーム名 ibis を引けず永久に解決せず、feedback が 1 パケットも出ない。
IBIS_TEAM_COLOR="${IBIS_TEAM_COLOR:-yellow}"
CRANE_TARGET_PORT="${CRANE_TARGET_PORT:-12345}"
# feedback は実機と同じ multicast で受ける。true にすると crane と cm4-sim が
# 127.0.0.1:50100+id を奪い合い、SO_REUSEPORT の振り分けは送信元を含む 4-tuple
# ハッシュで決まるため片方が全パケットを取る。crane 側が当たると cm4-sim は
# 位置信号を 1 つも受け取れず位置制御が死ぬ。
FEEDBACK_SIM_MODE="${FEEDBACK_SIM_MODE:-false}"

# crane -> cm4-sim 経路への劣化注入（無線区間の模擬）。
RX_DELAY_MS="${RX_DELAY_MS:-0}"
RX_JITTER_MS="${RX_JITTER_MS:-0}"
RX_LOSS_RATE="${RX_LOSS_RATE:-0.0}"
# 空なら compose 側の既定（commit SHA 固定）が使われる。
CM4_SIM_TAG="${CM4_SIM_TAG:-}"
CM4_ROBOT_IDS="${CM4_ROBOT_IDS:-0,1,2,3,4,5,6,7,8,9,10}"
CM4_RATE_HZ="${CM4_RATE_HZ:-1000}"
CM4_SEED="${CM4_SEED:-0}"

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
echo "経路: crane(${CRANE_TARGET_PORT}) -> cm4-sim -> simulator-cli(${IBIS_PORT})"
if [ "${FEEDBACK_SIM_MODE}" = "true" ]; then
    echo "feedback: unicast 127.0.0.1:50100+id"
else
    echo "feedback: multicast 224.5.20.(100+id):50100+id"
fi
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
# cm4-sim が 12345 を bind したまま残ると今回の構成と衝突する。
# --remove-orphans は旧 profile 構成の残骸も対象にする。
docker compose -f "${COMPOSE_FILE}" down --remove-orphans 2>/dev/null || true

# erforce-sim/auto-refereeはnetwork_mode:hostでマルチキャストを使うため、
# Wi-Fi/LANへの漏洩を防ぐホスト隔離設定を適用してから起動する
"${REPO_ROOT}/scripts/ensure-sim-network-confined.sh"

# Docker Composeでサービスを起動（simulator-cli / cm4-sim / auto-referee）
echo "Docker Composeでサービスを起動中..."
cd "${REPO_ROOT}"
CRANE_TAG="${CRANE_TAG}" PLANNER="${PLANNER}" \
    IBIS_PORT="${IBIS_PORT}" IBIS_TEAM_COLOR="${IBIS_TEAM_COLOR}" CRANE_TARGET_PORT="${CRANE_TARGET_PORT}" \
    FEEDBACK_SIM_MODE="${FEEDBACK_SIM_MODE}" \
    RX_DELAY_MS="${RX_DELAY_MS}" RX_JITTER_MS="${RX_JITTER_MS}" RX_LOSS_RATE="${RX_LOSS_RATE}" \
    CM4_SIM_TAG="${CM4_SIM_TAG}" CM4_ROBOT_IDS="${CM4_ROBOT_IDS}" \
    CM4_RATE_HZ="${CM4_RATE_HZ}" CM4_SEED="${CM4_SEED}" \
    docker compose -f "${COMPOSE_FILE}" up -d

# cm4-sim は位置制御ループそのものなので、落ちていればテストは無意味に失敗する。
# 原因が crane にあるように見えてしまうため、ここで早期に切り分ける。
CM4_STATE="$(docker inspect -f '{{.State.Running}}' cm4-sim 2>/dev/null || echo missing)"
if [ "${CM4_STATE}" != "true" ]; then
    echo "エラー: cm4-sim が起動していません (state=${CM4_STATE})" >&2
    docker compose -f "${COMPOSE_FILE}" logs cm4-sim || true
    docker compose -f "${COMPOSE_FILE}" down --remove-orphans || true
    exit 1
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
    setsid ros2 launch crane_bringup crane.launch.xml sim:=true speak:=false team:=Yellow planner:="${PLANNER}" >/tmp/crane_local.log 2>&1 &
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
    "--referee_port=11003"
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
docker compose -f "${COMPOSE_FILE}" logs

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

echo ""
echo "=== Docker Composeサービスを停止中 ==="
docker compose -f "${COMPOSE_FILE}" down

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
