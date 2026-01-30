#!/bin/bash

# TIGERs対戦ローカル実行スクリプト
#
# 使用方法:
#   ./scripts/match-vs-tigers/run_local.sh
#
# 注意:
#   - Dockerブリッジネットワークを使用するため、ホスト環境に影響しません
#   - craneもDocker内で実行されます（ホストで実行する場合はdocker-compose.local.yamlを使用）

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
COMPOSE_DIR="${PROJECT_ROOT}/docker/match-vs-tigers"

echo "========================================="
echo "  TIGERs対戦ローカル実行"
echo "========================================="
echo ""

# craneイメージのタグを確認
CRANE_TAG="${CRANE_TAG:-latest}"
echo "使用するcraneイメージ: ghcr.io/ibis-ssl/crane:${CRANE_TAG}"
echo ""

# 既存のコンテナをクリーンアップ
echo "既存のコンテナをクリーンアップ中..."
cd "${COMPOSE_DIR}"
docker compose down -v 2>/dev/null || true
echo "✓ クリーンアップ完了"
echo ""

# 結果ディレクトリをクリア
rm -rf "${COMPOSE_DIR}/results"
mkdir -p "${COMPOSE_DIR}/results"
echo "✓ 結果ディレクトリを作成"
echo ""

# Docker Composeサービスを起動
echo "Docker Composeサービスを起動中..."
export CRANE_TAG
docker compose up -d

echo "✓ サービスを起動しました"
echo ""

# match-controllerの完了を待機
echo "試合の完了を待機中..."
echo "（Ctrl+Cで中断できます）"
echo ""

while docker ps | grep -q match_controller; do
    sleep 5
done

echo "✓ 試合が完了しました"
echo ""

# 結果を表示
RESULT_FILE="${COMPOSE_DIR}/results/match_result.txt"
if [ -f "${RESULT_FILE}" ]; then
    echo "========================================="
    echo "  試合結果"
    echo "========================================="
    cat "${RESULT_FILE}"
    echo ""
else
    echo "✗ 結果ファイルが見つかりません: ${RESULT_FILE}"
    echo ""
fi

# ログを表示（オプション）
read -p "詳細ログを表示しますか？ (y/N): " -n 1 -r
echo ""
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo ""
    echo "========================================="
    echo "  Docker Composeログ"
    echo "========================================="
    docker compose logs
    echo ""
fi

# クリーンアップ
read -p "コンテナをクリーンアップしますか？ (Y/n): " -n 1 -r
echo ""
if [[ ! $REPLY =~ ^[Nn]$ ]]; then
    echo "コンテナをクリーンアップ中..."
    docker compose down -v
    echo "✓ クリーンアップ完了"
else
    echo "コンテナを維持します"
    echo "手動でクリーンアップする場合: cd ${COMPOSE_DIR} && docker compose down -v"
fi

echo ""
echo "========================================="
echo "  完了"
echo "========================================="
