#!/usr/bin/env bash
# シミュレータ利用時、SSL Vision/Referee/Tracker のマルチキャストがホスト内で
# 完結するように設定する（simモード専用。realモードでは使用しないこと）。
#
# 1. lo にマルチキャストを有効化し、224.0.0.0/4 の送信経路を lo に向ける。
#    network_mode: host で動く Docker コンテナ同士の multicast 通信を
#    ホスト内で完結させるための前提設定。
# 2. 224.5.23.0/24 (SSL Vision: 224.5.23.2, Game Controller: 224.5.23.1) について、
#    lo 以外のインターフェースへの送出を iptables で強制遮断する。
#    ssl-game-controller 等の一部ツールは送信元アドレスを各インターフェースのIPに
#    明示バインドしてマルチキャストを送信するため、1. のルーティング設定だけでは
#    Wi-Fi/LAN への漏洩を防げない（送信元バインドがルーティングテーブルより優先される）。
#    実際に Wi-Fi アクセスポイントが高頻度マルチキャストで過負荷になり落ちた実績があるため、
#    ルーティングとは独立してパケットフィルタでも確実に遮断する。
#    ホスト内の同一ネームスペースへの配送は IP_MULTICAST_LOOP により
#    物理インターフェースの状態に関わらず継続されるため、lo 以外への送出を
#    遮断するだけでコンテナ間通信は維持される。
#
# 冪等: すでに設定済みならスキップ。OS再起動やdockerネットワークの作り直しで失われるので
# 必要に応じて再実行すること。realモードで起動する際は scripts/restore-real-network.sh で
# 必ず解除すること（残っていると実機Vision/Refereeを受信できなくなる）。
set -euo pipefail

readonly MULTICAST_SUBNET="224.5.23.0/24"
readonly IPTABLES_COMMENT="crane-sim-multicast-confine"

# iptables -C はroot権限が無いと「ルール不在」ではなく Permission denied で失敗し、
# 判定を誤って sudo iptables -I が毎回実行され、遮断ルールが際限なく重複追加される。
# 誤判定を防ぐため、スクリプト全体をsudoで実行することを必須にする。
if [[ $EUID -ne 0 ]]; then
    echo "[setup-multicast] root権限が必要です。次のように実行してください: sudo $0" >&2
    exit 1
fi

need_multicast=false
if ! ip link show lo | grep -q MULTICAST; then
    need_multicast=true
fi

need_route=false
if ! ip route show 224.0.0.0/4 | grep -q "dev lo"; then
    need_route=true
fi

need_egress_block=false
if ! iptables -C OUTPUT -d "$MULTICAST_SUBNET" ! -o lo -m comment --comment "$IPTABLES_COMMENT" -j DROP 2>/dev/null; then
    need_egress_block=true
fi

if [[ $need_multicast == false && $need_route == false && $need_egress_block == false ]]; then
    echo "[setup-multicast] すでに設定済み (lo MULTICAST 有効 / 224.0.0.0/4 ルート済み / ${MULTICAST_SUBNET} 遮断済み)"
    exit 0
fi

echo "[setup-multicast] マルチキャスト設定を適用します (sudo が必要)"
if [[ $need_multicast == true ]]; then
    echo "  - lo に MULTICAST フラグを設定"
    sudo ip link set multicast on lo
fi
if [[ $need_route == true ]]; then
    echo "  - 224.0.0.0/4 を lo に向ける"
    sudo ip route replace 224.0.0.0/4 dev lo
fi
if [[ $need_egress_block == true ]]; then
    echo "  - ${MULTICAST_SUBNET} の lo 以外への送出を iptables で遮断"
    sudo iptables -I OUTPUT -d "$MULTICAST_SUBNET" ! -o lo -m comment --comment "$IPTABLES_COMMENT" -j DROP
fi
echo "[setup-multicast] 完了"
