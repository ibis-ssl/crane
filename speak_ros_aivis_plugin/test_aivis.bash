#!/bin/bash
# Aivis Speech Engine プラグインの動作確認スクリプト

set -e

echo "=== Aivis Speech Engine プラグイン テスト ==="
echo ""

# 1. Aivisエンジンの起動確認
echo "1. Aivisエンジンの起動を確認中..."
if curl -s http://localhost:10101/speakers > /dev/null 2>&1; then
    echo "   ✓ Aivisエンジンは起動しています"
else
    echo "   ✗ Aivisエンジンが起動していません"
    echo "   docker compose -f docker/sim/docker-compose.yaml up aivis-speech -d を実行してください"
    exit 1
fi

# 2. スピーカー一覧の表示
echo ""
echo "2. 利用可能なスピーカー:"
curl -s http://localhost:10101/speakers | python3 -c "
import sys, json
data = json.load(sys.stdin)
for speaker in data:
    print(f\"   - {speaker['name']}:\")
    for style in speaker['styles']:
        print(f\"     ID {style['id']}: {style['name']}\")
" 2>/dev/null || echo "   (スピーカー情報の取得に失敗)"

# 3. プラグインのビルド確認
echo ""
echo "3. プラグインのビルド確認..."
if [ -f "$COLCON_PREFIX_PATH/speak_ros_aivis_plugin/lib/libspeak_ros_aivis_plugin.so" ]; then
    echo "   ✓ プラグインライブラリが存在します"
else
    echo "   ✗ プラグインがビルドされていません"
    echo "   colcon build --packages-select speak_ros_aivis_plugin を実行してください"
    exit 1
fi

echo ""
echo "=== テスト完了 ==="
echo ""
echo "次のコマンドでspeak_rosノードを起動できます:"
echo "  ros2 run speak_ros speak_ros_node --ros-args -p plugin_name:=aivis_plugin::AivisPlugin"
echo ""
echo "音声合成のテスト:"
echo "  ros2 action send_goal /speak speak_ros_interfaces/action/Speak \\"
echo "    \"{text: 'こんにちは、Aivisです。', speed_rate: 1.0}\""
