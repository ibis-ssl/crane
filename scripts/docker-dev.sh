#!/bin/bash
# Docker開発環境の起動スクリプト
# Usage:
#   ./scripts/docker-dev.sh [sim|real] [docker-compose-args...]
#
# Examples:
#   ./scripts/docker-dev.sh           # sim環境で起動（フォアグラウンド）
#   ./scripts/docker-dev.sh -d        # sim環境で起動（バックグラウンド）
#   ./scripts/docker-dev.sh real      # real環境で起動
#   ./scripts/docker-dev.sh real -d   # real環境で起動（バックグラウンド）
#   ./scripts/docker-dev.sh down      # 停止

set -e

# スクリプトのあるディレクトリからリポジトリルートへ移動
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$REPO_ROOT"

COMPOSE_FILE="docker/dev/docker-compose.yaml"

# 引数解析
MODE="sim"
DOCKER_ARGS=()

if [[ $# -gt 0 ]]; then
    if [[ $1 == "real" ]]; then
        MODE="real"
        shift
    elif [[ $1 == "sim" ]]; then
        MODE="sim"
        shift
    fi
fi

# 残りの引数をDocker Composeに渡す
DOCKER_ARGS=("$@")

echo "=== Docker開発環境 ==="
echo "モード: $MODE"
echo "Compose file: $COMPOSE_FILE"
echo "引数: ${DOCKER_ARGS[*]}"
echo ""

if [[ $MODE == "sim" ]]; then
    # シミュレーション環境（status-board有効）
    docker compose -f "$COMPOSE_FILE" --profile sim "${DOCKER_ARGS[@]}"
else
    # 実機環境（Visionポート変更、status-board無効）
    VISION_PORT=10006 docker compose -f "$COMPOSE_FILE" "${DOCKER_ARGS[@]}"
fi
