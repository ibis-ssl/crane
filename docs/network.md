# ネットワークと実機通信

パケット形式、宛先とポート、停止判定、cm4-sim の構成は [ibis-ssl/specs](https://github.com/ibis-ssl/specs)（リポジトリ直下の `specs/` サブモジュール）が正本です。このページには Crane 側の設定・手順・制約だけを置きます。

- [宛先とポート](https://github.com/ibis-ssl/specs/blob/main/common/system_overview.md#機体番号とネットワーク) / [制御パケット](https://github.com/ibis-ssl/specs/blob/main/common/control_packet.md) / [位置制御設定パケット](https://github.com/ibis-ssl/specs/blob/main/common/config_packet.md) / [フィードバックパケット](https://github.com/ibis-ssl/specs/blob/main/common/feedback_packet.md)
- [停止判定](https://github.com/ibis-ssl/specs/blob/main/common/safety_stop.md) / [シミュレーション（cm4-sim）](https://github.com/ibis-ssl/specs/blob/main/pc/simulation.md) / [crane との連携](https://github.com/ibis-ssl/specs/blob/main/pc/crane_integration.md)

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

指令の宛先は `ibis_sender_node` のパラメータ `unicast_to_robot_ip` で切り替わり、`sim:=false` では各機体の CM4、`sim:=true` では全機体を `127.0.0.1` の `cm4-sim` へ送ります。フィードバックは実機・シミュレーションとも常に multicast で受けます。

## ロボット側の位置制御

位置制御ループはシステム全体で 1 つだけで、ロボットの CM4（シミュレーションでは `cm4-sim`）が閉じます。Crane 側に位置制御ループを作らないでください。Crane は位置指令を送るだけなので、無線区間は制御ループの外側に出ます。制御則と既定値は [CM4 の位置制御](https://github.com/ibis-ssl/specs/blob/main/cm4/position_control.md) を参照してください。

| プランナー | ROS 制御モード | ワイヤ制御モード | CM4 / `cm4-sim` の動作 |
|---|---|---|---|
| `visibility_graph` | `POSITION_TARGET_MODE` (1) | mode 4 | 位置制御ループを回す |
| `rvo2` | `POLAR_VELOCITY_TARGET_MODE` (3) | mode 3 | 位置制御せずそのまま転送 |

### 起動

```bash
# シミュレーション（sim-erforce プロファイルで erforce-sim と cm4-sim が起動する）
./scripts/docker-dev.sh
ros2 launch crane_bringup crane.launch.xml sim:=true planner:=visibility_graph

# 実機
./scripts/docker-dev.sh real
ros2 launch crane_bringup crane.launch.xml sim:=false planner:=visibility_graph
```

`cm4-sim` が使う unicast のフィードバックポートは、観測のためでも別プロセスで bind しないでください。配送が片方に偏り、位置制御が効かなくなります。観測は再配信先の multicast で行います（理由は [シミュレーションの注意点](https://github.com/ibis-ssl/specs/blob/main/pc/simulation.md#注意点)）。

### ゲインの設定

[ibis_sender_node](https://github.com/ibis-ssl/crane/blob/develop/crane_sender/src/ibis_sender_node.cpp) は 1 秒ごとにパラメータを読み直し、[位置制御設定パケット](https://github.com/ibis-ssl/specs/blob/main/common/config_packet.md)を送ります。受理範囲を外れた値はデータグラムごと捨てられるので、範囲は specs で確認してください。

- `position_control.kp` / `ki` / `kd`: PID ゲイン（`ki` / `kd` の既定は `0.0` で P 制御）。移動中は速度上限に張り付いて積分が止まるので、`ki` は到達後の定常偏差を消すためのもの。追従を速くしたいときは `kp` を上げる
- `position_control.deceleration`: 最大減速度 [m/s²]
- `position_control.tolerance`: 目標到達の許容誤差 [m]
- `position_control.config_port`: 宛先ポート

稼働中に `ros2 param set` で変えれば再起動なしで反映されます。CM4 は値が変わったときだけ `位置制御の設定を更新: kp ... / ki ... / kd ...` をログに出すので、反映はこのログで確認します。

```bash
ros2 param set /ibis_sender position_control.kp 2.5
ros2 param get /ibis_sender position_control.kp
```

設定パケットの形式は crane と CM4 で完全一致を要求されます。形式を変えたときは、Orion_CM4 側でイメージを発行したあと、`docker/dev/docker-compose.yaml`、`docker/scenario/docker-compose.yaml`（`.local.yaml` を含む）、`docker/match-vs-tigers/docker-compose.yaml` の `CM4_SIM_TAG` 既定値を更新します。更新しないと `cm4-sim` がゲイン設定を全数拒否します（[Docker イメージ](https://github.com/ibis-ssl/specs/blob/main/pc/simulation.md#docker-イメージ)）。

### 送った値の記録

CM4 は ACK を返さないので、bag から送信内容を追えるよう、送信ノードは次のトピックを publish します。どちらも `record:=true` の記録対象です。

- `/position_control_config`（[`crane_msgs/msg/PositionControlConfig`](https://github.com/ibis-ssl/crane/blob/develop/crane_msgs/msg/PositionControlConfig.msg)）: 設定パケットと同じ内容。送信と同じ経路なので、`/robot_commands` が流れている間だけ最大 1 Hz で出ます。vision が落ちて world model が更新されない区間は送信も publish も止まり、トピックが途切れます。現在値はパラメータを引いてください。
- `/sent_robot_commands`（`crane_msgs/msg/RobotCommands`）: 送信直後の指令。`/robot_commands` に含まれない値を `planning_factors`（`SenderVisionAvailable` / `SenderWireMode` / `SenderLinearVelocityLimit` / `SenderAccelerationLimit` / `SenderElapsedVisionMs`）に、UDP 送信の成否を `SenderSent`（1/0）に載せます。world model を一度も受信していない起動直後を除き `/robot_commands` の 1 フレームにつき 1 件出るので、`/sent_robot_commands` に無いフレームは送信ノードが処理していない、`SenderSent=0` のフレームは送信に失敗した、と切り分けられます。

送信に失敗した場合も値は `sent: false` として残ります。`sent: true` や `SenderSent=1` はソケットへ渡せたことだけを示し、ロボットが受け取った証拠ではありません。

```bash
# bag に記録されたゲインを時刻つきで読む（echo を先に起動してから再生する）
ros2 topic echo /position_control_config &
ros2 bag play --rate 10 <bag>
```

## 動かないとき

[診断](diagnostics.md)を確認し、Craneの指令生成、UDP送信、CM4の受信・UART中継、G474の解釈、フィードバックの順に切り分けます。届いているのに動作が違う場合は、モード・座標・単位・パケット互換性を確認します。

位置制御が効かないときは次も確認します。

1. シミュレーションでは `docker ps` で `cm4-sim` が `Up` か。
2. `planner:=visibility_graph` を指定しているか（`rvo2` は mode 3 を出す）。
3. `docker compose -f docker/dev/docker-compose.yaml logs cm4-sim` にパケット破棄や設定の拒否（`位置制御の設定パケットを拒否しました: WrongSize` など）が出ていないか。

## 通信仕様を変更するとき

`RobotCommandSerializedV2` のレイアウトの正本は Crane の [robot_packet.h](https://github.com/ibis-ssl/crane/blob/develop/crane_sender/include/crane_sender/robot_packet.h) です。Orion_CM4・G474・framework との対応と一致検査は [正本と一致検査](https://github.com/ibis-ssl/specs/blob/main/common/control_packet.md#正本と一致検査) を参照してください。

1. 各リポジトリのヘッダーを同期し、構造体サイズ・配置・エンディアン、制御モード、[座標・単位](coordinates.md)の解釈を確認する。
2. Crane 側では [crane_packet_forge](https://github.com/ibis-ssl/crane/tree/develop/crane_packet_forge) の `layout.py` と基準ベクタを再生成する。
3. 各プログラムをビルドし、実際の受信・制御・フィードバック、タイミングと欠損時の挙動を統合テストする。
4. specs の該当文書を更新する。

ヘッダーにモードが定義されていても、実機側で制御が実装されているとは限りません。[局所経路計画の実機制約](rvo2_local_planner.md)も確認してください。
