#!/bin/bash
# debug_tools (crane_websocket_server) をバックグラウンドで起動するスクリプト
# Usage:
#   ./scripts/start-debug-tools.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
DEBUG_TOOLS_PID_FILE="/tmp/crane_debug_tools.pid"

# 既に起動している場合はスキップ
if [[ -f $DEBUG_TOOLS_PID_FILE ]]; then
    old_pid=$(cat "$DEBUG_TOOLS_PID_FILE")
    if kill -0 "$old_pid" 2>/dev/null; then
        echo "debug_tools は既に起動中です (PID: $old_pid)"
        exit 0
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

echo "debug_tools 起動完了"
echo "  HTTP Server: http://localhost:8090"
echo "  WebSocket: ws://localhost:8091"
echo "  PID: $(cat $DEBUG_TOOLS_PID_FILE)"
echo ""
echo "停止するには: ./scripts/stop-debug-tools.sh"
