#!/usr/bin/env bash
# シミュレータ系Docker Compose起動スクリプト(scenario_test, match-vs-tigers等)から
# 共通で呼び出す、マルチキャスト隔離(scripts/setup-multicast.sh)の適用ラッパー。
#
# docker-dev.sh --minimal 使用時に、Vision/Refereeマルチキャストが実際にWi-Fi経由で
# 漏洩しアクセスポイントを過負荷で落とした実績があるため、host networkでSSLシミュレータ
# 系コンテナ(erforce-sim/grsim + ssl-game-controller等)を起動する経路は全て
# この隔離設定を適用してから起動すること。
#
# 常に適用を試み、失敗したら起動を中断する（fail-closed）。sudoのパスワード入力が
# できない非対話環境で意図的にスキップしたい場合のみ、CRANE_ALLOW_MULTICAST_LEAK=1 を
# 明示的に指定すること（現状このリポジトリのCIはこの経路を通らないため、通常は不要）。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ ${CRANE_ALLOW_MULTICAST_LEAK:-0} == "1" ]]; then
    echo "⚠️  CRANE_ALLOW_MULTICAST_LEAK=1 が指定されているため、マルチキャスト隔離設定を明示的にスキップします。" >&2
    echo "    (Wi-Fi/LANへのマルチキャスト漏洩を防げません。物理Wi-Fiが無い使い捨て環境以外では指定しないこと)" >&2
    exit 0
fi

if ! sudo "${SCRIPT_DIR}/setup-multicast.sh"; then
    echo "❌ マルチキャスト隔離設定に失敗したため、ネットワーク保護のため起動を中断します。" >&2
    echo "    sudoのパスワード入力ができない非対話環境の場合は、環境変数" >&2
    echo "    CRANE_ALLOW_MULTICAST_LEAK=1 を明示的に指定することでこのチェックをスキップできます" >&2
    echo "    (Wi-Fi/LANへの漏洩を防げなくなるため、物理Wi-Fiの無い環境以外では非推奨)。" >&2
    exit 1
fi
