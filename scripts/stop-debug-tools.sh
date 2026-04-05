#!/bin/bash
# robot-manager コンテナを停止するスクリプト
# Usage:
#   ./scripts/stop-debug-tools.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
COMPOSE_FILE="$REPO_ROOT/docker/dev/docker-compose.yaml"

echo "=== robot-manager を停止中 ==="
docker compose -f "$COMPOSE_FILE" stop robot-manager
echo "robot-manager 停止完了"
