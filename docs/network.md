# ネットワークと実機通信

## シミュレーションの隔離

host network のシミュレータは、起動前にマルチキャストをホスト内へ隔離します。物理Wi-Fi/LANへの漏洩でアクセスポイントが過負荷になった事例があります。

リポジトリルートで手動適用する場合:

```bash
sudo ./scripts/setup-multicast.sh
```

[setup-multicast.sh](https://github.com/ibis-ssl/crane/blob/develop/scripts/setup-multicast.sh) は、ループバックのマルチキャスト有効化・経路設定と、物理インターフェースへのSSLマルチキャスト送出の遮断を行います。送信元をインターフェースに束縛するツールがあるため、経路設定だけでは不十分です。

Docker開発・シナリオ・対戦テストの起動スクリプトは隔離を適用し、失敗時は起動を中断します。Composeを直接起動するときも、この前提を満たしてください。

再起動やファイアウォール設定変更後は再確認します。Game Controllerの起動直後だけの `operation not permitted` は遮断による場合がありますが、`Sending UDP datagram failed` や `messageGen unresponsive` が連続する場合は経路・隔離状態を確認して再適用します。シミュレーション中に遮断ルールだけを外さないでください。

## 実機へ切り替える

1. シミュレータとローカルのGame Controllerを停止する。
2. `./scripts/docker-dev.sh real` を使う。Crane用の遮断ルールを解除し、ループバック向けマルチキャスト経路が残っていれば起動を中断する。
3. 経路の警告が出た場合は、他用途のDDS通信への影響を確認し、スクリプトが表示する解除手順を実行してから再起動する。大会ネットワーク側のインターフェースへ経路を合わせる。
4. `sim:=false` で起動し、[試合チェック](match.md)で実際の受信と指令を確認する。

解除の正本は [restore-real-network.sh](https://github.com/ibis-ssl/crane/blob/develop/scripts/restore-real-network.sh)。単に `sim:=false` を指定してもホストの隔離設定は解除されません。

## 接続先を確認する

| 通信 | 設定・実装 |
|---|---|
| Vision / Referee / Tracker、送信先の起動設定 | [crane.launch.xml](https://github.com/ibis-ssl/crane/blob/develop/crane_bringup/launch/crane.launch.xml) |
| ロボットへの送信形式・宛先 | [ibis_sender_node.cpp](https://github.com/ibis-ssl/crane/blob/develop/crane_sender/src/ibis_sender_node.cpp) |
| ロボットのフィードバック受信 | [crane_robot_receiver](https://github.com/ibis-ssl/crane/tree/develop/crane_robot_receiver) |
| Docker側のサービス接続 | [開発環境のCompose](https://github.com/ibis-ssl/crane/blob/develop/docker/dev/docker-compose.yaml) |

送受信側のアドレス・ポート・チーム設定を合わせ、起動ログと使用中の設定で確認します。`cm4_sim` を挟む構成では `feedback_sim_mode:=false` が必要です。同じunicastポートをCraneとブリッジが受信すると、片方だけにパケットが配送されるためです。

## 通信仕様を変更するとき

実機指令は Crane → UDP → [Orion_CM4](https://github.com/ibis-ssl/Orion_CM4) → UART → [G474_Orion_main](https://github.com/ibis-ssl/G474_Orion_main) と渡ります。

1. Craneの [robot_packet.h](https://github.com/ibis-ssl/crane/blob/develop/crane_sender/include/crane_sender/robot_packet.h) を基準に、Orion_CM4の `robot_packet.h` とG474の `Core/Inc/robot_packet.h` を同期する。
2. 構造体サイズ・配置・エンディアン、制御モード、[座標・単位](coordinates.md)の解釈を全プログラムで確認する。UART設定はCM4とG474で合わせる。
3. 各プログラムをビルドし、実際の受信・制御・フィードバック、タイミングと欠損時の挙動を統合テストする。

ヘッダーにモードが定義されていても、実機側で制御が実装されているとは限りません。[局所経路計画の実機制約](rvo2_local_planner.md)も確認してください。

## 動かないとき

[診断](diagnostics.md)を確認し、Craneの指令生成、UDP送信、CM4の受信・UART中継、G474の解釈、フィードバックの順に切り分けます。届いているのに動作が違う場合は、モード・座標・単位・パケット互換性を確認します。
