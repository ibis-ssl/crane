#!/usr/bin/env bash
# ホスト側でマルチキャストが通るように lo を有効化し 224.0.0.0/4 をループバックへ向ける。
# network_mode: host で動く Docker コンテナ同士の multicast 通信（SSL Vision/Referee/Tracker など）を
# ホスト内で完結させるための前提設定。OS 再起動で失われるので必要に応じて再実行する。
#
# 冪等: すでに設定済みならスキップ。
set -euo pipefail

need_multicast=false
if ! ip link show lo | grep -q MULTICAST; then
    need_multicast=true
fi

need_route=false
if ! ip route show 224.0.0.0/4 | grep -q "dev lo"; then
    need_route=true
fi

if [[ $need_multicast == false && $need_route == false ]]; then
    echo "[setup-multicast] すでに設定済み (lo MULTICAST 有効 / 224.0.0.0/4 ルート済み)"
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
echo "[setup-multicast] 完了"
