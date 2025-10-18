#!/bin/bash
# GitHub Codespaces 専用の高速セットアップスクリプト
set -e

echo "======================================"
echo "Crane Codespaces 高速セットアップ"
echo "======================================"
echo ""
echo "ℹ️  このスクリプトは GitHub Codespaces 専用です"
echo "ℹ️  最小限のセットアップのみ実行し、起動時間を短縮します"
echo ""

# ワークスペースルートに移動
cd /home/developer/ibis_ws

# ROS 環境の読み込み
source /opt/ros/jazzy/setup.bash

# Codespaces 環境かチェック
if [ -z "${CODESPACES}" ]; then
    echo "⚠️  このスクリプトは Codespaces 環境専用です"
    echo "   ローカル開発環境では postCreate.sh を使用してください"
    exit 0
fi

echo "[1/4] 依存パッケージのインポート中..."
if [ -f src/crane/dependency_jazzy.repos ]; then
    # 既存のパッケージは更新しない（開発中の変更を保護）
    vcs import src < src/crane/dependency_jazzy.repos --skip-existing
    echo "✓ 依存パッケージのインポート完了"
else
    echo "⚠ dependency_jazzy.repos が見つかりません。スキップします。"
fi

echo ""
echo "[2/4] システム依存関係のインストール中..."
rosdep update
rosdep install -r -y -i --from-paths src --rosdistro jazzy
echo "✓ システム依存関係のインストール完了"

echo ""
echo "[3/4] pre-commit フックのインストール中..."
cd src/crane
if [ -f .pre-commit-config.yaml ]; then
    pre-commit install --install-hooks
    echo "✓ pre-commit フックのインストール完了"
else
    echo "⚠ .pre-commit-config.yaml が見つかりません。スキップします。"
fi
cd /home/developer/ibis_ws

echo ""
echo "[4/4] Git 設定..."
git config --global core.autocrlf input 2>/dev/null || true
git config --global --add safe.directory /home/developer/ibis_ws 2>/dev/null || true
echo "✓ Git 設定完了"

echo ""
echo "======================================"
echo "高速セットアップ完了！"
echo "======================================"
echo ""
echo "⚡ Codespaces 最適化により、フルビルドは省略されました"
echo ""
echo "次のステップ:"
echo "  1. ビルド: colcon build --symlink-install"
echo "     （初回ビルド: 約5〜10分）"
echo ""
echo "  2. 環境読み込み: source install/local_setup.bash"
echo ""
echo "  3. テスト: colcon test --event-handlers console_cohesion+"
echo ""
echo "💡 ヒント:"
echo "  - VS Code タスクを使用: Ctrl+Shift+B"
echo "  - 特定パッケージのみビルド: colcon build --packages-select <package>"
echo "  - 最適化ビルド: ./src/crane/scripts/optimized_build.bash"
echo ""
echo "📚 詳細情報: src/crane/.devcontainer/README.md"
echo ""
