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

送受信側のアドレス・ポート・チーム設定を合わせ、起動ログと使用中の設定で確認します。

## シミュレーションの標準構成

実機CM4に相当する `cm4-sim` が経路に入り、位置制御ループを閉じます。Craneは位置指令（ワイヤmode 4）を送るだけで、不安定な無線経路に相当する区間が制御ループの外側に出ます。

```text
crane --12345 mode4--> cm4-sim --12346 mode3--> simulator-cli
  ^                       ^                          |
  |                       +-- unicast feedback 127.0.0.1:50100+id --+
  +-- multicast feedback 224.5.20.(100+id):50100+id（cm4-simが再配信）--+
```

mode 4 を出すのは `planner:=visibility_graph` だけです。`rvo2` は mode 3 を出し、`cm4-sim` はそれを位置制御せずそのまま転送します。

Crane は常にフィードバックを multicast 側で受信するよう設定されています。同じ unicast ポートを Crane と `cm4-sim` が受信すると、`SO_REUSEPORT` の振り分けは送信元を含む 4-tuple ハッシュで決まるため、片方だけに全パケットが配送されます。Crane 側が当たると `cm4-sim` は位置信号を受け取れず、位置制御が動きません。Crane を multicast 受信にすることで、unicast は `cm4-sim` が独占できます。

同じ理由で、feedbackを観測したいときに `cm4-sim` と同じunicastポート（`--feedback-port-base` が示す `127.0.0.1:50100+id`）を別プロセスでbindしてはいけません。配送が片方に偏り、「位置制御が効いていない」ように見えます。観測は再配信先の `224.5.20.(100+id):50100+id` で行います。再配信自体は `cm4_sim --no-feedback-relay` で止められます。

詳しいアーキテクチャ・起動手順・制約は [CM4・cm4-simでの位置制御](cm4_position_control.md)、サービス定義は[シナリオ用Compose](https://github.com/ibis-ssl/crane/blob/develop/docker/scenario/docker-compose.yaml)です。

## 通信仕様を変更するとき

実機指令は Crane → UDP → [Orion_CM4](https://github.com/ibis-ssl/Orion_CM4) → UART → [G474_Orion_main](https://github.com/ibis-ssl/G474_Orion_main) と渡ります。

1. Craneの [robot_packet.h](https://github.com/ibis-ssl/crane/blob/develop/crane_sender/include/crane_sender/robot_packet.h) を基準に、Orion_CM4の `robot_packet.h` とG474の `Core/Inc/robot_packet.h` を同期する。
2. 構造体サイズ・配置・エンディアン、制御モード、[座標・単位](coordinates.md)の解釈を全プログラムで確認する。UART設定はCM4とG474で合わせる。
3. 各プログラムをビルドし、実際の受信・制御・フィードバック、タイミングと欠損時の挙動を統合テストする。

ヘッダーにモードが定義されていても、実機側で制御が実装されているとは限りません。[局所経路計画の実機制約](rvo2_local_planner.md)も確認してください。

## 動かないとき

[診断](diagnostics.md)を確認し、Craneの指令生成、UDP送信、CM4の受信・UART中継、G474の解釈、フィードバックの順に切り分けます。届いているのに動作が違う場合は、モード・座標・単位・パケット互換性を確認します。
