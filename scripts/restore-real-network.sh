#!/usr/bin/env bash
# realモード起動前に、simモード用のマルチキャスト隔離設定が残っていないことを確認する。
#
# scripts/setup-multicast.sh は sim 用に以下を設定する:
#   (a) 224.0.0.0/4 の送信経路を lo に強制
#   (b) 224.5.23.0/24 (SSL Vision/Referee/Tracker) と 224.5.20.0/24 (ロボット feedback) の
#       lo 以外への送出を iptables で遮断
#
# これが real モードでも残っていると、実機の Vision/Referee を物理インターフェースから
# 受信・送信できずサイレントに機能しなくなる（試合中に気づけないと重大事故になる）。
#
# (b) は crane-sim-multicast-confine というコメント付きで本リポジトリのスクリプトだけが
# 作成するルールなので、検出したら自動的に削除する（対象は224.5.23.0/24のみでROS 2 DDSの
# デフォルトアドレス 239.255.0.1 には影響しないため、安全に自動削除できる）。
# (a) の 224.0.0.0/4 ルートは対象範囲が広く、ROS 2 DDS (239.255.0.1) を含むホスト内
# マルチキャスト全般に影響する。docs/network.md では「DDS含むホスト内マルチキャストの手動設定」
# としても案内されており、本スクリプト由来か利用者が別の目的で設定したものか区別できない。
# 実機動作に必須の解除ではあるが、意図しないユーザー設定を無断で消さないよう
# ここでは自動削除せず、警告した上で利用者に対応を委ねる（起動は中断する）。
#
# 冪等: sim用の設定が残っていなければ何もしない。
set -euo pipefail

readonly MULTICAST_SUBNETS=("224.5.23.0/24" "224.5.20.0/24")
readonly IPTABLES_COMMENT="crane-sim-multicast-confine"

# iptables -C はroot権限が無いと「ルール不在」ではなく Permission denied で失敗し、
# 「sim用設定は残っていない」と誤判定してしまう（realモードで一番あってはならない誤り）。
# 誤判定を防ぐため、スクリプト全体をsudoで実行することを必須にする。
if [[ $EUID -ne 0 ]]; then
    echo "[restore-real-network] root権限が必要です。次のように実行してください: sudo $0" >&2
    exit 1
fi

changed=false
route_warning=false

for subnet in "${MULTICAST_SUBNETS[@]}"; do
    while iptables -C OUTPUT -d "$subnet" ! -o lo -m comment --comment "$IPTABLES_COMMENT" -j DROP 2>/dev/null; do
        echo "[restore-real-network] simモード用の遮断ルール (${subnet}) を解除します"
        sudo iptables -D OUTPUT -d "$subnet" ! -o lo -m comment --comment "$IPTABLES_COMMENT" -j DROP
        changed=true
    done
done

if ip route show 224.0.0.0/4 2>/dev/null | grep -q "dev lo"; then
    route_warning=true
fi

if [[ $changed == true ]]; then
    echo "[restore-real-network] simモード用の遮断ルールを解除しました。"
fi

if [[ $route_warning == true ]]; then
    echo "======================================================================" >&2
    echo "⚠️  【警告】224.0.0.0/4 の送信経路が lo（ループバック）に向いています。" >&2
    echo "    このままでは実機の Vision/Referee を物理インターフェースで" >&2
    echo "    送受信できず、サイレントに機能しません。" >&2
    echo "    このルートは docs/network.md 記載の手動設定 (ROS 2 DDS 等) と共通のため" >&2
    echo "    このスクリプトからは自動削除しません。以下を実行して解除してください:" >&2
    echo "" >&2
    echo "      sudo ip route del 224.0.0.0/4 dev lo" >&2
    echo "" >&2
    echo "======================================================================" >&2
    exit 1
fi

if [[ $changed == false ]]; then
    echo "[restore-real-network] simモード用の設定は残っていません (変更なし)"
fi
