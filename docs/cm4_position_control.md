# CM4・cm4-simでの位置制御

Crane は、無線通信の遅延やジッターの影響を抑えて高い追従性を実現するため、位置制御ループをロボット側（CM4）およびシミュレータ中間プロセス（`cm4-sim`）で閉じるアーキテクチャを採用しています。

## 目的と設計理由

従来の方式では、ホストPC上の Crane が位置フィードバックを受けて速度指令（mode 3）を計算し、Wi-Fi 経由でロボットへ送信していました。この構成では、無線ネットワークの遅延・ジッター・パケット欠損が位置制御ループ（閉ループ）の内側に含まれ、急加減速時の振動や追従遅れの原因となります。

ロボット側位置制御（Robot-side Position Control）では、Crane は目標位置や終端速度を含む位置指令（ワイヤ mode 4）を送信し、ロボット側の CM4 が高周期（1000 Hz）で位置制御ループを閉じます。

- **不変条件**: システム全体で位置制御ループは**ちょうど1つ**です。Crane は `packet_type=ibis` の経路で位置制御（二重ループ）を行いません。
- **無線区間の分離**: 不安定な無線通信区間が制御ループの外側に出るため、通信に多少のジッターや遅延があっても安定した位置追従を維持できます。

## システム構成とデータフロー

### シミュレーション構成（cm4-sim）

実機の CM4 に相当するコンテナ `cm4-sim` が Crane とシミュレータ（`simulator-cli`）の間に入り、位置制御ループを閉じます。

```text
crane --(UDP:12345 mode 4)--> cm4-sim --(UDP:12346 mode 3)--> simulator-cli
  ^                             ^                                  |
  |                             +-- unicast: 127.0.0.1:50100+id ---+
  +-- multicast: 224.5.20.(100+id):50100+id (cm4-sim が再配信)----+
```

- **Crane 送信先**: ポート `12345`（`cm4-sim` の入力ポート）
- **cm4-sim 出力先**: ポート `12346`（`simulator-cli` の ibis 入力ポート）
- **フィードバック**: `simulator-cli` から `cm4-sim` へ unicast（`127.0.0.1:50100+id`）で届き、`cm4-sim` が multicast（`224.5.20.(100+id):50100+id`）へ再配信して Crane が受信します。

### 実機構成（Orion_CM4）

実機では、ロボットに搭載された Raspberry Pi CM4（[Orion_CM4](https://github.com/ibis-ssl/Orion_CM4)）が位置制御ループを実行します。

```text
Crane --(Wi-Fi Broadcast:12345 mode 4)--> CM4 --(UART)--> G474 (モータ制御)
  |                                        ^
  +--(Wi-Fi Broadcast:12350 ゲイン設定)----+
```

- **Crane 送信先**: ブロードキャスト `192.168.20.255:12345`
- **設定送信先**: ブロードキャスト `192.168.20.255:12350`
- **制御の流れ**: CM4 が位置制御器で速度を計算し、下位マイコン（G474）へ UART で送信します。

## 起動手順

### シミュレーション環境

1. **Docker 開発環境の起動**
   `./scripts/docker-dev.sh` を実行します（`sim-erforce` プロファイルで `erforce-sim` と `cm4-sim` が起動します）。

   ```bash
   ./scripts/docker-dev.sh
   ```

2. **Crane の起動**
   `planner:=visibility_graph`、`ibis_target_port:=12345`、および **`feedback_sim_mode:=false`** を指定して起動します。

   ```bash
   ros2 launch crane_bringup crane.launch.xml sim:=true planner:=visibility_graph ibis_target_port:=12345 feedback_sim_mode:=false
   ```

### 実機環境

1. **ネットワークの切り替え**
   実機向けスクリプトで隔離ルールを解除して起動します。

   ```bash
   ./scripts/docker-dev.sh real
   ```

2. **Crane の起動**
   `sim:=false` で起動します（`ibis_target_port` は既定値 `12345`、ブロードキャストアドレス `192.168.20.255` に設定されます）。

   ```bash
   ros2 launch crane_bringup crane.launch.xml sim:=false planner:=visibility_graph
   ```

## プランナーと制御モード

| プランナー | ROS 制御モード | ワイヤ制御モード | 動作 |
|---|---|---|---|
| `visibility_graph` | `POSITION_TARGET_MODE` (1) | mode 4 (`POSITION_TARGET_WITH_TERMINAL_VELOCITY_MODE`) | CM4 / `cm4-sim` が位置制御ループを回す |
| `rvo2` | `POLAR_VELOCITY_TARGET_MODE` (3) | mode 3 (`POLAR_VELOCITY_TARGET_MODE`) | CM4 / `cm4-sim` は位置制御を行わずそのまま転送（パススルー） |

## 守るべき制約（ハマりどころ）

### 1. `feedback_sim_mode:=false` の必須性

シミュレーションで `cm4-sim` を挟む場合は、Crane 起動時に **`feedback_sim_mode:=false`** の指定が必須です。

- **理由**: `sim:=true` の既定値では `feedback_sim_mode:=true`（unicast 受信）となります。Crane と `cm4-sim` の両方が同一ポート `127.0.0.1:50100+id` を `SO_REUSEPORT` でバインドすると、Linux カーネルの 4-tuple ハッシュにより単一送信元（`simulator-cli`）からのパケットが片方に全量偏って配送されます。Crane 側に当たると `cm4-sim` は位置信号を 1 パケットも受け取れず、位置制御が停止します。
- `feedback_sim_mode:=false` を指定することで、Crane は multicast 側（`224.5.20.(100+id):50100+id`）で受信し、unicast ポートは `cm4-sim` が排他的に利用できます。

### 2. ユニキャストポートの重複バインド禁止

デバッグ時であっても、`cm4-sim` が使用中の unicast ポート（`127.0.0.1:50100+id`）を別プロセスや解析スクリプトでバインドしてはいけません。フィードバックを観測したい場合は、再配信先の multicast（`224.5.20.(100+id):50100+id`）を購読してください。

## パラメータ設定と動的更新

Crane の [ibis_sender_node](https://github.com/ibis-ssl/crane/blob/develop/crane_sender/src/ibis_sender_node.cpp) は、1 秒ごとに位置制御設定パケット（20 バイト固定）をポート `12350` へブロードキャストします。

### パラメータ一覧

- `position_control.kp`: 位置比例ゲイン
- `position_control.deceleration`: 減速時の最大減速度
- `position_control.tolerance`: 目標到達と判定する許容誤差距離
- `position_control.config_port`: 設定パケットの宛先ポート（既定: `12350`）

### 稼働中のパラメータ変更

設定値は 1 秒ごとに読み直して送信されるため、`ros2 param set` で変更すればロボットやノードを再起動せずに即座に反映されます。

```bash
ros2 param set /ibis_sender_node position_control.kp 2.5
```

## トラブルシューティング

ロボットが動かない、あるいは位置制御が効かない場合は以下の順で確認します。

1. **`cm4-sim` コンテナの状態確認（シミュレーション時）**
   `docker ps` で `cm4-sim` が `Up` になっているか確認します。
2. **送信先ポートの確認**
   Crane 側の `ibis_target_port` が `12345`（`cm4-sim` 入力）になっているか確認します。誤って `12346`（シミュレータ直受け）に送っていないか確認してください。
3. **`feedback_sim_mode:=false` の確認**
   起動時引数に `feedback_sim_mode:=false` が含まれているか確認します。抜けているとフィードバックの奪い合いが発生します。
4. **プランナーの確認**
   位置制御を行う場合は `planner:=visibility_graph` を指定しているか確認します。`rvo2` の場合は速度指令（mode 3）となります。
5. **診断ログの確認**
   [診断](diagnostics.md) および `docker compose -f docker/dev/docker-compose.yaml logs cm4-sim` でエラーやパケット破棄が出ていないか確認します。

## 実装リファレンス

- 送信ノード・パケット生成: [ibis_sender_node.cpp](https://github.com/ibis-ssl/crane/blob/develop/crane_sender/src/ibis_sender_node.cpp)
- パケット定義: [robot_packet.h](https://github.com/ibis-ssl/crane/blob/develop/crane_sender/include/crane_sender/robot_packet.h)
- シミュレータ位置制御近似: [sim_position_controller.cpp](https://github.com/ibis-ssl/crane/blob/develop/crane_sender/src/sim_position_controller.cpp)
- コンテナ構成・ポート定義: [docker-compose.yaml](https://github.com/ibis-ssl/crane/blob/develop/docker/dev/docker-compose.yaml)
- 起動引数定義: [crane.launch.xml](https://github.com/ibis-ssl/crane/blob/develop/crane_bringup/launch/crane.launch.xml)
- 局所経路計画: [rvo2_local_planner.md](rvo2_local_planner.md)
- 通信仕様: [network.md](network.md)
