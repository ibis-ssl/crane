#!/bin/bash
set -e

echo "======================================"
echo "Crane DevContainer 初期セットアップ"
echo "======================================"

# ワークスペースルートに移動
cd /home/developer/ibis_ws

# ROS 環境の読み込み
source /opt/ros/jazzy/setup.bash

echo ""
echo "[1/6] 依存パッケージのインポート中..."
if [ -f src/crane/dependency_jazzy.repos ]; then
    # 既存のパッケージは更新しない（開発中の変更を保護）
    vcs import src --skip-existing <src/crane/dependency_jazzy.repos
    echo "✓ 依存パッケージのインポート完了"
else
    echo "⚠ dependency_jazzy.repos が見つかりません。スキップします。"
fi

echo ""
echo "[2/6] システム依存関係のインストール中..."
rosdep update
rosdep install -r -y -i --from-paths src --rosdistro jazzy
echo "✓ システム依存関係のインストール完了"

echo ""
echo "[3/6] ワークスペースのビルド中..."
# 初回ビルドは symlink-install で実行
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
echo "✓ ビルド完了"

echo ""
echo "[4/6] ROS 環境の読み込み..."
source install/local_setup.bash
echo "✓ ROS 環境の読み込み完了"

echo ""
echo "[5/6] pre-commit フックのインストール中..."
cd src/crane
if [ -f .pre-commit-config.yaml ]; then
    pre-commit install
    echo "✓ pre-commit フックのインストール完了"
else
    echo "⚠ .pre-commit-config.yaml が見つかりません。スキップします。"
fi
cd /home/developer/ibis_ws

echo ""
echo "[6/6] Git 設定の確認..."
# Git の改行設定を確認
git config --global core.autocrlf input 2>/dev/null || true
echo "✓ Git 設定完了"

echo ""
echo "======================================"
echo "セットアップ完了！"
echo "======================================"
echo ""
echo "次のステップ:"
echo "  1. シミュレーション起動: ros2 launch crane_bringup crane.launch.py sim:=true"
echo "  2. ビルド: colcon build --symlink-install"
echo "  3. テスト: colcon test --event-handlers console_cohesion+"
echo ""
echo "便利なコマンド:"
echo "  - 最適化ビルド: ./src/crane/scripts/optimized_build.bash"
echo "  - クリーンビルド: ./src/crane/scripts/optimized_build.bash clean"
echo ""
echo "Web インターフェース:"
echo "  - Game Controller: http://localhost:8081"
echo "  - Vision Client:   http://localhost:8082"
echo "  - Status Board:    http://localhost:8083"
echo ""
