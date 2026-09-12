#!/bin/bash
# Docker開発環境の起動スクリプト
# Usage:
#   ./scripts/docker-dev.sh [sim|real] [--sim erforce|grsim] [--minimal] [--robot-manager|--no-debug] [docker-compose-args...]
#
# Examples:
#   ./scripts/docker-dev.sh                     # sim環境(ER-Force)
#   ./scripts/docker-dev.sh --minimal           # 最小構成(シミュレータ + GCのみ)
#   ./scripts/docker-dev.sh --sim grsim         # sim環境(grSim)
#   ./scripts/docker-dev.sh -d                  # sim環境(バックグラウンド)
#   ./scripts/docker-dev.sh --robot-manager     # robot-managerあり(sim環境)
#   ./scripts/docker-dev.sh down                # 停止

set -e

# スクリプトのあるディレクトリからリポジトリルートへ移動
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$REPO_ROOT"

COMPOSE_FILE="docker/dev/docker-compose.yaml"

# 引数解析
MODE="sim"
SIM="erforce"
ENABLE_ROBOT_MANAGER=""
MINIMAL=false
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
    --sim)
        SIM="$2"
        if [[ $SIM != "erforce" && $SIM != "grsim" ]]; then
            echo "エラー: --sim には erforce または grsim を指定してください" >&2
            exit 1
        fi
        shift 2
        ;;
    --minimal)
        MINIMAL=true
        shift
        ;;
    --no-debug)
        ENABLE_ROBOT_MANAGER=false
        shift
        ;;
    --robot-manager | --debug)
        ENABLE_ROBOT_MANAGER=true
        shift
        ;;
    *)
        DOCKER_ARGS+=("$1")
        shift
        ;;
    esac
done

# robot-managerのデフォルト: simモード時は不要なネットワーク負荷を避けるため無効、realモード時は有効
if [[ -z $ENABLE_ROBOT_MANAGER ]]; then
    if [[ $MODE == "sim" ]]; then
        ENABLE_ROBOT_MANAGER=false
    else
        ENABLE_ROBOT_MANAGER=true
    fi
fi

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

# マルチキャスト設定のチェック＆ブロック実行 (up時のみ)
if [[ $COMPOSE_COMMAND == "up" ]]; then
    if ! ip link show lo 2>/dev/null | grep -q MULTICAST || ! ip route show 224.0.0.0/4 2>/dev/null | grep -q "dev lo"; then
        echo "======================================================================" >&2
        echo "⚠️  【警告】マルチキャスト設定が lo（ループバック）に向けられていません！" >&2
        echo "    このまま起動すると、Wi-Fi/LANにマルチキャストパケットが漏洩し、" >&2
        echo "    ネットワーク帯域が著しく圧迫されます。" >&2
        echo "    マルチキャスト設定を適用します (sudo パスワードの入力が必要です)..." >&2
        echo "======================================================================" >&2
        if sudo "$REPO_ROOT/scripts/setup-multicast.sh"; then
            echo "✅ マルチキャスト設定が完了しました。" >&2
            echo "" >&2
        else
            echo "❌ マルチキャスト設定に失敗したため、ネットワーク保護のため起動を中断します。" >&2
            exit 1
        fi
    fi
fi

echo "=== Docker開発環境 ==="
echo "モード: $MODE"
if [[ $MODE == "sim" ]]; then
    echo "シミュレータ: $SIM"
fi
echo "最小構成 (--minimal): $MINIMAL"
echo "robot-manager: $ENABLE_ROBOT_MANAGER"
echo "compose command: $COMPOSE_COMMAND"
echo "Compose file: $COMPOSE_FILE"
echo "引数: ${DOCKER_ARGS[*]}"
echo ""

# 明示的なサブコマンドが DOCKER_ARGS にない場合、先頭に COMPOSE_COMMAND を補完
HAS_SUBCOMMAND=false
for arg in "${DOCKER_ARGS[@]}"; do
    case "$arg" in
    up | down | build | ps | stop | restart | logs | config | exec | run | pull | create | kill | rm | top)
        HAS_SUBCOMMAND=true
        break
        ;;
    esac
done

if [[ $HAS_SUBCOMMAND == "false" ]]; then
    DOCKER_ARGS=("$COMPOSE_COMMAND" "${DOCKER_ARGS[@]}")
fi

if [[ $ENABLE_ROBOT_MANAGER == "false" ]] && [[ $COMPOSE_COMMAND == "up" ]] && [[ $MINIMAL == "false" ]]; then
    DOCKER_ARGS+=(--scale robot-manager=0)
fi

# 最小構成モード: シミュレータ本体、Game Controller、Vision Client (http://localhost:8082) のみを起動
if [[ $MINIMAL == "true" ]] && [[ $COMPOSE_COMMAND == "up" ]]; then
    if [[ $MODE == "sim" ]]; then
        if [[ $SIM == "erforce" ]]; then
            DOCKER_ARGS+=("erforce-sim" "ssl-game-controller" "ssl-vision-client")
        else
            DOCKER_ARGS+=("grsim" "ssl-game-controller" "ssl-vision-client")
        fi
    else
        DOCKER_ARGS+=("ssl-game-controller" "ssl-vision-client")
    fi
fi

case "$COMPOSE_COMMAND" in
build | up | create)
    "$REPO_ROOT/docker/dev/ball-calibration/scripts/sync_proto.sh"
    ;;
esac

if [[ $MODE == "sim" ]]; then
    # シミュレーション環境(status-board有効)
    docker compose -f "$COMPOSE_FILE" --profile "sim-${SIM}" "${DOCKER_ARGS[@]}"
else
    # 実機環境(Visionポート変更、status-board無効)
    VISION_PORT=10006 REFEREE_PORT=10003 TRACKER_PORT=10010 docker compose -f "$COMPOSE_FILE" "${DOCKER_ARGS[@]}"
fi
