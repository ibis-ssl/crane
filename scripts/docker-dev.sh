#!/bin/bash
# Docker開発環境の起動スクリプト
# Usage:
#   ./scripts/docker-dev.sh [sim|real] [--no-debug] [docker-compose-args...]
#
# Examples:
#   ./scripts/docker-dev.sh           # sim環境 + debug_tools
#   ./scripts/docker-dev.sh -d        # sim環境(バックグラウンド) + debug_tools
#   ./scripts/docker-dev.sh --no-debug  # debug_toolsなし
#   ./scripts/docker-dev.sh down      # 停止(debug_toolsも停止)

set -e

# スクリプトのあるディレクトリからリポジトリルートへ移動
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$REPO_ROOT"

COMPOSE_FILE="docker/dev/docker-compose.yaml"
DEBUG_TOOLS_PID_FILE="/tmp/crane_debug_tools.pid"

# 引数解析
MODE="sim"
ENABLE_DEBUG_TOOLS=true
DOCKER_ARGS=()

while [[ $# -gt 0 ]]; do
    case $1 in
    real)
        MODE="real"
        shift
        ;;
    sim)
        MODE="sim"
        shift
        ;;
    --no-debug)
        ENABLE_DEBUG_TOOLS=false
        shift
        ;;
    *)
        DOCKER_ARGS+=("$1")
        shift
        ;;
    esac
done

start_debug_tools() {
    if [[ -f $DEBUG_TOOLS_PID_FILE ]]; then
        local old_pid
        old_pid=$(cat "$DEBUG_TOOLS_PID_FILE")
        if kill -0 "$old_pid" 2>/dev/null; then
            echo "debug_tools は既に起動中です (PID: $old_pid)"
            return 0
        fi
    fi

    echo "=== debug_tools を起動中 ==="
    # ROS2環境をセットアップしてノードを起動
    # shellcheck disable=SC1090,SC1091,SC2086
    (
        source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
        source "$REPO_ROOT/../../install/setup.bash" 2>/dev/null || true
        ros2 run crane_debug_tools crane_websocket_server \
            --ros-args -p port:=8090 -p websocket_port:=8091 &
        echo $! >"$DEBUG_TOOLS_PID_FILE"
    )
    echo "debug_tools 起動完了 (HTTP: 8090, WebSocket: 8091)"
}

stop_debug_tools() {
    if [[ -f $DEBUG_TOOLS_PID_FILE ]]; then
        local pid
        pid=$(cat "$DEBUG_TOOLS_PID_FILE")
        if kill -0 "$pid" 2>/dev/null; then
            echo "=== debug_tools を停止中 (PID: $pid) ==="
            kill "$pid" 2>/dev/null || true
            rm -f "$DEBUG_TOOLS_PID_FILE"
        fi
    fi
}

# downコマンドの場合はdebug_toolsも停止
if [[ " ${DOCKER_ARGS[*]} " =~ " down " ]]; then
    stop_debug_tools
fi

echo "=== Docker開発環境 ==="
echo "モード: $MODE"
echo "debug_tools: $ENABLE_DEBUG_TOOLS"
echo "Compose file: $COMPOSE_FILE"
echo "引数: ${DOCKER_ARGS[*]}"
echo ""

if [[ $MODE == "sim" ]]; then
    # シミュレーション環境(status-board有効)
    docker compose -f "$COMPOSE_FILE" --profile sim "${DOCKER_ARGS[@]}"
else
    # 実機環境(Visionポート変更、status-board無効)
    VISION_PORT=10006 docker compose -f "$COMPOSE_FILE" "${DOCKER_ARGS[@]}"
fi

# バックグラウンド起動時のみdebug_toolsを起動
if [[ $ENABLE_DEBUG_TOOLS == "true" ]] && [[ " ${DOCKER_ARGS[*]} " =~ " -d " ]]; then
    start_debug_tools
fi
