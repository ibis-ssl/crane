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
   `planner:=visibility_graph` を指定して起動します（送信先ポート `12345`、アドレス `127.0.0.1`、multicast フィードバック受信は `sim:=true` により自動設定されます）。

   ```bash
   ros2 launch crane_bringup crane.launch.xml sim:=true planner:=visibility_graph
   ```

### 実機環境

1. **ネットワークの切り替え**
   実機向けスクリプトで隔離ルールを解除して起動します。

   ```bash
   ./scripts/docker-dev.sh real
   ```

2. **Crane の起動**
   `sim:=false` で起動します（送信先ポート `12345`、ブロードキャストアドレス `192.168.20.255` に自動設定されます）。

   ```bash
   ros2 launch crane_bringup crane.launch.xml sim:=false planner:=visibility_graph
   ```

## プランナーと制御モード

| プランナー | ROS 制御モード | ワイヤ制御モード | 動作 |
|---|---|---|---|
| `visibility_graph` | `POSITION_TARGET_MODE` (1) | mode 4 (`POSITION_TARGET_WITH_TERMINAL_VELOCITY_MODE`) | CM4 / `cm4-sim` が位置制御ループを回す |
| `rvo2` | `POLAR_VELOCITY_TARGET_MODE` (3) | mode 3 (`POLAR_VELOCITY_TARGET_MODE`) | CM4 / `cm4-sim` は位置制御を行わずそのまま転送（パススルー） |

## 守るべき制約（ハマりどころ）

### 1. multicast フィードバック受信の前提

シミュレーションで `cm4-sim` を挟む場合、Crane 側はフィードバックを multicast（`224.5.20.(100+id):50100+id`）で受信する必要があります（`crane.launch.xml` で常にこの設定に固定されています）。

- **理由**: Crane と `cm4-sim` の両方が同一ポート `127.0.0.1:50100+id` を `SO_REUSEPORT` でバインドすると、Linux カーネルの 4-tuple ハッシュにより単一送信元（`simulator-cli`）からのパケットが片方に全量偏って配送されます。Crane 側に当たると `cm4-sim` は位置信号を 1 パケットも受け取れず、位置制御が停止します。
- Crane が multicast 側で受信することで、unicast ポートは `cm4-sim` が排他的に利用できます。

### 2. ユニキャストポートの重複バインド禁止

デバッグ時であっても、`cm4-sim` が使用中の unicast ポート（`127.0.0.1:50100+id`）を別プロセスや解析スクリプトでバインドしてはいけません。フィードバックを観測したい場合は、再配信先の multicast（`224.5.20.(100+id):50100+id`）を購読してください。

## パラメータ設定と動的更新

Crane の [ibis_sender_node](https://github.com/ibis-ssl/crane/blob/develop/crane_sender/src/ibis_sender_node.cpp) は、1 秒ごとに位置制御設定パケット（28 バイト固定、v2）をポート `12350` へブロードキャストします。

CM4 側の制御則は **PID** です。ただし `ki` / `kd` の既定は `0` で、その場合は従来どおりの P 制御に恒等的に縮退します。PID を使うときは現地で `ki` / `kd` を上げてください。

### パラメータ一覧

- `position_control.kp`: 位置比例ゲイン [1/s]
- `position_control.ki`: 積分ゲイン [1/s²]（既定 `0.0` = P 制御）。床の摩擦差やスリップで残る定常偏差を消すために使います
- `position_control.kd`: 微分ゲイン [無次元]（既定 `0.0` = P 制御）。誤差ではなく実測速度に掛かる微分先行形なので、目標更新のたびに微分キックが出ることはありません
- `position_control.deceleration`: 減速時の最大減速度 [m/s²]
- `position_control.tolerance`: 目標到達と判定する許容誤差距離 [m]
- `position_control.config_port`: 設定パケットの宛先ポート（既定: `12350`）

`ki` / `kd` が効くのは `packet_type=ibis` の経路（実機および `cm4-sim` 構成）だけです。`packet_type=ssl` 経路の位置制御（[sim_position_controller.cpp](https://github.com/ibis-ssl/crane/blob/develop/crane_sender/src/sim_position_controller.cpp)）は P 制御のままで、これらの値は送信も参照もされません。

### 範囲外の値は捨てられる

CM4 は受信した値をクランプせず、**データグラムごと破棄**して拒否理由をログに出します（黙ってクランプすると crane の表示と実機の実効値が食い違ったまま気付けないため）。検査はデータグラム単位なので、`ki` だけが範囲外でも `kp` を含めて 1 つも適用されません。

| パラメータ | 受理範囲 |
|---|---|
| `kp` | `0 <= v <= 20` |
| `ki` | `0 <= v <= 20` |
| `kd` | `0 <= v <= 5` |
| `deceleration` | `0 <= v <= 20` |
| `tolerance` | `0 <= v <= 1.0` |

### 後方互換はありません

設定パケットは v2（28 バイト）のみです。CM4 は旧フォーマット（v1・20 バイト）を受理せず、crane 側も v2 しか送りません。中途半端に互換を残すと「`kp` だけ効いて `ki` / `kd` が効いていない機体」が黙って混ざり、現地では「なんとなく追従が悪い」以外の症状が出ないためです。

片方だけ古いと設定パケットは `WrongSize` として全数拒否され、その機体は停止するのではなく**既定ゲイン（`kp = 2.0`）のまま走り続けます**（CM4 のログには拒否理由が出ます）。

> [!IMPORTANT]
> **リリース順序**: この変更は Orion_CM4 側の更新とセットです。crane だけ、あるいは CM4 だけを配ってはいけません。
>
> 1. Orion_CM4 の PID 対応をマージし、`ghcr.io/ibis-ssl/orion-cm4-sim` のイメージを発行する
> 2. `docker/dev/docker-compose.yaml` と `docker/scenario/docker-compose.yaml` の `CM4_SIM_TAG` 既定値（commit SHA）を新しいイメージへ更新する
> 3. 実機の CM4 へ新しいバイナリを配る（`cm4-fleet deploy`）
>
> 2 を飛ばすと、`cm4-sim` を挟むシミュレーションでゲイン設定が全数拒否され、`kp` の変更すら一切効かなくなります（`docker compose -f docker/dev/docker-compose.yaml logs cm4-sim` に `位置制御の設定パケットを拒否しました: WrongSize` が出ます）。

### `ki` が効かないように見えるとき

移動中はほぼ常に速度上限（減速エンベロープ）に張り付いており、その間 CM4 は**積分を進めません**（ワインドアップ抑制）。`ki` は目標へ詰めきったあとに残る定常偏差を消すためのもので、移動中の追従を速くするものではありません。追従そのものを速くしたい場合は `kp` を上げてください。

### 稼働中のパラメータ変更

設定値は 1 秒ごとに読み直して送信されるため、`ros2 param set` で変更すればロボットやノードを再起動せずに即座に反映されます。

```bash
ros2 param set /ibis_sender_node position_control.kp 2.5
ros2 param set /ibis_sender_node position_control.ki 1.0
ros2 param set /ibis_sender_node position_control.kd 0.1
```

CM4 側は値が変わったときだけ `位置制御の設定を更新: kp ... / ki ... / kd ...` を出力します。反映されたかはこのログで確認してください。

## トラブルシューティング

ロボットが動かない、あるいは位置制御が効かない場合は以下の順で確認します。

1. **`cm4-sim` コンテナの状態確認（シミュレーション時）**
   `docker ps` で `cm4-sim` が `Up` になっているか確認します。
2. **プランナーの確認**
   位置制御を行う場合は `planner:=visibility_graph` を指定しているか確認します。`rvo2` の場合は速度指令（mode 3）となります。
3. **診断ログの確認**
   [診断](diagnostics.md) および `docker compose -f docker/dev/docker-compose.yaml logs cm4-sim` でエラーやパケット破棄が出ていないか確認します。

## 実装リファレンス

- 送信ノード・パケット生成: [ibis_sender_node.cpp](https://github.com/ibis-ssl/crane/blob/develop/crane_sender/src/ibis_sender_node.cpp)
- パケット定義: [robot_packet.h](https://github.com/ibis-ssl/crane/blob/develop/crane_sender/include/crane_sender/robot_packet.h)
- シミュレータ位置制御近似: [sim_position_controller.cpp](https://github.com/ibis-ssl/crane/blob/develop/crane_sender/src/sim_position_controller.cpp)
- コンテナ構成・ポート定義: [docker-compose.yaml](https://github.com/ibis-ssl/crane/blob/develop/docker/dev/docker-compose.yaml)
- 起動引数定義: [crane.launch.xml](https://github.com/ibis-ssl/crane/blob/develop/crane_bringup/launch/crane.launch.xml)
- 局所経路計画: [rvo2_local_planner.md](rvo2_local_planner.md)
- 通信仕様: [network.md](network.md)
