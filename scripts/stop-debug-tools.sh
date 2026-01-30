#!/bin/bash
# debug_tools (crane_websocket_server) を停止するスクリプト
# Usage:
#   ./scripts/stop-debug-tools.sh

set -e

DEBUG_TOOLS_PID_FILE="/tmp/crane_debug_tools.pid"

if [[ ! -f $DEBUG_TOOLS_PID_FILE ]]; then
    echo "debug_tools は起動していません (PIDファイルが見つかりません)"
    exit 0
fi

pid=$(cat "$DEBUG_TOOLS_PID_FILE")

if ! kill -0 "$pid" 2>/dev/null; then
    echo "debug_tools は既に停止しています (PID: $pid は存在しません)"
    rm -f "$DEBUG_TOOLS_PID_FILE"
    exit 0
fi

echo "=== debug_tools を停止中 (PID: $pid) ==="
kill "$pid" 2>/dev/null || true

# プロセスが完全に終了するまで待機(最大5秒)
for _ in {1..10}; do
    if ! kill -0 "$pid" 2>/dev/null; then
        break
    fi
    sleep 0.5
done

# 強制終了が必要な場合
if kill -0 "$pid" 2>/dev/null; then
    echo "プロセスが終了しないため、強制終了します..."
    kill -9 "$pid" 2>/dev/null || true
fi

rm -f "$DEBUG_TOOLS_PID_FILE"
echo "debug_tools 停止完了"
