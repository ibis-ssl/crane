#!/bin/bash
# Docker開発環境の起動スクリプト
# Usage:
#   ./scripts/docker-dev.sh [sim|real] [--no-debug] [docker-compose-args...]
#
# Examples:
#   ./scripts/docker-dev.sh           # sim環境 + ssl-log-recorder
#   ./scripts/docker-dev.sh -d        # sim環境(バックグラウンド)
#   ./scripts/docker-dev.sh --no-debug  # robot-managerなし
#   ./scripts/docker-dev.sh down      # 停止

set -e

# スクリプトのあるディレクトリからリポジトリルートへ移動
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$REPO_ROOT"

COMPOSE_FILE="docker/dev/docker-compose.yaml"

# 引数解析
MODE="sim"
ENABLE_ROBOT_MANAGER=true
DOCKER_ARGS=()

while [[ $# -gt 0 ]]; do
    case $1 in
    real)
        MODE="real"
        shift
        ;;
    sim)
        MODE="sim"
        shift
        ;;
    --no-debug)
        ENABLE_ROBOT_MANAGER=false
        shift
        ;;
    *)
        DOCKER_ARGS+=("$1")
        shift
        ;;
    esac
done

detect_compose_command() {
    # docker compose グローバルオプションをスキップしてサブコマンドを特定する
    local skip_next=false
    local arg

    for arg in "${DOCKER_ARGS[@]}"; do
        if [[ $skip_next == "true" ]]; then
            skip_next=false
            continue
        fi

        case "$arg" in
        --file | -f | --project-name | -p | --profile | --project-directory | --env-file | --parallel | --progress | --ansi)
            skip_next=true
            continue
            ;;
        --file=* | --project-name=* | --profile=* | --project-directory=* | --env-file=* | --parallel=* | --progress=* | --ansi=*)
            continue
            ;;
        --*)
            continue
            ;;
        -*)
            continue
            ;;
        *)
            echo "$arg"
            return 0
            ;;
        esac
    done

    # サブコマンド未指定の場合は docker compose up のデフォルト挙動
    echo "up"
}

COMPOSE_COMMAND="$(detect_compose_command)"

echo "=== Docker開発環境 ==="
echo "モード: $MODE"
echo "robot-manager: $ENABLE_ROBOT_MANAGER"
echo "compose command: $COMPOSE_COMMAND"
echo "Compose file: $COMPOSE_FILE"
echo "引数: ${DOCKER_ARGS[*]}"
echo ""

if [[ $ENABLE_ROBOT_MANAGER == "false" ]] && [[ $COMPOSE_COMMAND == "up" ]]; then
    DOCKER_ARGS+=(--scale robot-manager=0)
fi

if [[ $MODE == "sim" ]]; then
    # シミュレーション環境(status-board有効)
    docker compose -f "$COMPOSE_FILE" --profile sim "${DOCKER_ARGS[@]}"
else
    # 実機環境(Visionポート変更、status-board無効)
    VISION_PORT=10006 REFEREE_PORT=10003 TRACKER_PORT=10010 docker compose -f "$COMPOSE_FILE" "${DOCKER_ARGS[@]}"
fi
