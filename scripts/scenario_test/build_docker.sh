#!/bin/bash
# シナリオテスト用Dockerイメージのビルドスクリプト

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

IMAGE_NAME="ghcr.io/ibis-ssl/crane"
IMAGE_TAG="${CRANE_TAG:-local-scenario}"
FULL_IMAGE="${IMAGE_NAME}:${IMAGE_TAG}"

echo "=== シナリオテスト用Dockerイメージのビルドを開始 ==="
echo "イメージ: ${FULL_IMAGE}"
echo ""

cd "${REPO_ROOT}"

# buildxのセットアップ確認
if ! docker buildx version &> /dev/null; then
    echo "警告: docker buildxが利用できません。通常のdocker buildを使用します。"
    docker build \
        -f docker/scenario/Dockerfile \
        -t "${FULL_IMAGE}" \
        .
else
    # buildxを使用したビルド（キャッシュ最適化）
    docker buildx build \
        -f docker/scenario/Dockerfile \
        -t "${FULL_IMAGE}" \
        --load \
        .
fi

echo ""
echo "=== イメージのビルドが完了しました ==="
echo "イメージ: ${FULL_IMAGE}"
echo ""
echo "このイメージを使用してテストを実行するには："
echo "  CRANE_TAG=${IMAGE_TAG} make scenario-test"
echo ""
