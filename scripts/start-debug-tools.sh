#!/bin/bash
# robot-manager コンテナを起動するスクリプト
# Usage:
#   ./scripts/start-debug-tools.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
COMPOSE_FILE="$REPO_ROOT/docker/dev/docker-compose.yaml"

echo "=== robot-manager を起動中 ==="
docker compose -f "$COMPOSE_FILE" up -d robot-manager
echo "robot-manager 起動完了"
echo "  URL: http://localhost:8090"
echo ""
echo "停止するには: ./scripts/stop-debug-tools.sh"
