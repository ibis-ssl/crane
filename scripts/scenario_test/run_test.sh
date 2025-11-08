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

# Docker Composeでサービスを起動（grSimとauto-referee）
echo "Docker Composeでサービスを起動中..."
cd "${REPO_ROOT}"
CRANE_TAG="${CRANE_TAG}" docker compose -f "${COMPOSE_FILE}" up -d

# ローカルモードの場合、craneをローカルで起動
CRANE_PID=""
if [ "${USE_LOCAL}" = "1" ]; then
    echo "ローカル環境でcraneを起動中..."
    cd "${WORKSPACE_ROOT}"

    # ROS 2環境のセットアップとcraneの起動（バックグラウンド）
    # shellcheck source=/dev/null
    source "${WORKSPACE_ROOT}/install/setup.bash"
    ros2 launch crane_bringup crane.launch.xml sim:=true speak:=false vision_port:=10020 referee_port:=10003 team:=Yellow >/tmp/crane_local.log 2>&1 &
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

# 動画生成（テスト失敗時のみ）
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

    # 動画の生成
    cd "${REPO_ROOT}"
    for logfile in *.log.gz; do
        if [ -f "${logfile}" ]; then
            echo "動画を生成中: ${logfile}"
            gunzip -c "${logfile}" >"${logfile%.gz}"
            "${SSL_LOG_VIDEO_DIR}/cmd/ssl-log-video/ssl-log-video" -file "${logfile%.gz}" -output "${logfile%.gz}.avi"
            ffmpeg -i "${logfile%.gz}.avi" -vcodec libx264 -acodec aac "${logfile%.gz}.mp4" -y
            echo "動画を生成しました: ${logfile%.gz}.mp4"
        fi
    done
fi

# クリーンアップ
echo ""

# ローカルモードの場合、craneプロセスを停止
if [ -n "${CRANE_PID}" ]; then
    echo "=== ローカルcraneプロセスを停止中 ==="
    # プロセスグループ全体を終了（子プロセスも含む）
    kill -TERM -${CRANE_PID} 2>/dev/null || true
    sleep 2
    # まだ残っている場合は強制終了
    kill -KILL -${CRANE_PID} 2>/dev/null || true
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
