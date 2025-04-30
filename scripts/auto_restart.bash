#!/bin/bash

# スクリプトの位置を取得
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# workspace のルートディレクトリを計算（scripts から3階層上がる）
WORKSPACE_DIR="$(dirname "$(dirname "$(dirname "$SCRIPT_DIR")")")"

# setup.bash のパスを構築
SETUP_BASH="${WORKSPACE_DIR}/install/setup.bash"

COMMAND="source ${SETUP_BASH} && ros2 launch crane_bringup crane.launch.py"

echo "コマンドを実行します: ${COMMAND}"

while true; do
    # コマンドを実行
    bash -c "${COMMAND}"

    # コマンドの終了ステータスを取得
    EXIT_STATUS=$?

    echo "コマンドがコード $EXIT_STATUS で終了しました。"
    echo "ros関連プロセスをクリーンアップします"
    ps aux | pgrep ros | grep -v grep | awk '{ print "kill -9 " $2 }' | sh
    echo "2秒待ってリスタートします。リスタートする前にCtrl+Cを実行すると終了できます"
    sleep 2
done

echo "自動再実行スクリプトを終了します"
