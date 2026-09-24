# crane_packet_forge

ロボット指令パケットを自由に組み立てて、crane を経由せず直接送るツールです。CLI と GUI があります。

## 何のためにあるか

crane の既存経路はすべて [sender_base.cpp](https://github.com/ibis-ssl/crane/blob/develop/crane_sender/src/sender_base.cpp)
の `SenderBase::callback` を通り、そこで world_model 未更新なら送信そのものが止まり、
`latency_ms`・`elapsed_time_ms_since_last_vision`・`is_vision_available` は上書きされ、
`kick_power` はクランプされます。Web UI の Robot Test も Move モードも、
セッション → スキル → ローカルプランナー → sender を経由します。

「このバイト列をこの周期で投げたらロボットがどう振る舞うか」を試すには、
その層より下で組み立てる必要があります。このパッケージはそこを担当します。
crane が動いていなくても（むしろ動いていないほうが良い状態で）使えます。

## 安全について

**上限で拒否しません。** 危険な組み合わせは警告として出すだけです。
実機を動かす前に、台に載せる・車輪を浮かせる・周囲を空けるといった確認は人間が行ってください。

既定で有効な動作（いずれも外せます）:

- `check_counter` の 0..200 巡回（`--freeze-counter` で固定）。これは安全装置ではなくプロトコルです。
  250ms 変化しないと受信側は AI 断とみなします
- 終了時に速度 0 + `STOP_EMERGENCY` を 5 回送る（`--no-stop-on-exit` で無効、
  `--abort-after` は crane が停止指令なしで落ちた状況の再現用）
- crane の `ibis_sender_node` が動いていたら警告。2 つの送信元が同じ CM4 へ送ると
  `check_counter` が入り乱れ、**測定結果そのものが壊れます**

## CLI

```bash
# 何が送られるか確認する（ソケットを開かない）
./crane-forge send --robot 7 --set control_mode=3 \
    --set polar.target_global_velocity_r=0.3 --dry-run

# 実際に送る（Ctrl-C か --duration まで）
./crane-forge send --robot 7 --set control_mode=3 \
    --set polar.target_global_velocity_r=0.3 \
    --set flags.is_vision_available=1 --duration 10

# 指定できるフィールドの一覧（オフセット・レンジ・注意書きつき）
./crane-forge fields

# 区間スケジュール（8秒停止 → 2秒 0.2m/s → 8秒停止）
./crane-forge send --robot 7 --set flags.is_vision_available=1 \
    --schedule 'polar.target_global_velocity_r=0:8;\
polar.target_global_velocity_r=0.2:2;polar.target_global_velocity_r=0:8'

# エージェントから使うとき: stdout が JSONL になる
./crane-forge send --spec spec.json --json

# 受け取ったバイト列を読む
./crane-forge decode --hex "00 07 7f ff ..."

# フィードバックを multicast で受ける
./crane-forge watch --robot 7 --json

# GUI のバックエンド
./crane-forge serve --port 8094
```

ワークスペースを source していれば `ros2 run crane_packet_forge crane-forge ...` でも同じです。

失敗は終了コードで区別します（2: 引数・spec 不正 / 3: ネットワーク / 130: 中断）。

### packet spec

CLI と GUI が共有する唯一の入力形式です。`--spec` で読み、GUI は同じ JSON を送ります。
キーは `--set` のキーと一字一句同じです。

```json
{
  "target": "real",
  "robot_id": 7,
  "rate_hz": 62.5,
  "base": "neutral",
  "fields": {
    "control_mode": 3,
    "polar.target_global_velocity_r": 0.3,
    "flags.is_vision_available": true
  },
  "raw_bytes": { "37": 255 },
  "steps": [{ "duration_s": 8, "fields": { "polar.target_global_velocity_r": 0 } }]
}
```

`base` は未指定フィールドの埋め方です。`neutral`（既定）は 2 バイト値に **0.0 を符号化** します。
`zeros` は生ゼロのままで、これは crane の空きスロットと同じ状態ですが、
**0x0000 は 0 ではなく -range**（速度リミットなら -32.767）を意味します。

`raw_bytes` はフィールドより後に適用されるので常に勝ちます。

## GUI

`http://<host>:8094/`。crane_web_debugger のナビから開けます。
docker では `docker/dev/docker-compose.yaml` の `packet-forge` サービスが起動します。
ホストで直接動かす場合はフォントが無いのでアイコンが文字で出ます
（`FONTS_DIR` を指定するか `crane_web_debugger/web/download_fonts.py` で取得してください）。

- 左: フィールド。クリックで座標、ドラッグで `target_global_theta`
- 中央: フィールド編集。フラグは **8 ビットすべて独立**（`robot_packet.h` が名前を付けていない
  bit 2・4-7 も個別に立てられます）、連続値はスライダー + 数値入力、`control_mode` はセグメント
- 右: 64 バイトの hex グリッド。クリックで生バイト上書き
- 下: フィードバック帯。送信中の `check_counter` が返ってきているかを並べて表示します
- `steps` ボタン: 区間スケジュール。時間で値を切り替えます（CLI の `--schedule` と同じもの）。
  送信中は実行中の区間が強調されます

`spec` ボタンから JSON の入出力と、同じ内容の CLI コマンドを取り出せます。

## 実機以外での確かめ方

crane を起動せずに、送信・受信の両方をループバックで確認できます。

```bash
# 送ったバイト列を、robot_packet.h とは別に書かれたデコーダで読む
python3 ../scenario_test/dump_ibis_packets.py --bind 127.0.0.1 --port 12345 --expect-mode 3
./crane-forge send --target 127.0.0.1 --robot 7 --duration 1 \
    --set control_mode=3 --set polar.target_global_velocity_r=0.3

# フィードバックの受信側（合成パケットを流して確かめる）
python3 ../scenario_test/inject_feedback.py --robot-id 7 --duration 10 --multicast-if 127.0.0.1
./crane-forge watch --robot 7 --interface-ip 127.0.0.1
```

## 知っておくこと

- **`flags.is_vision_available` が 0 だと G474 は出力しません。** 無指令のキープアライブには
  これで正しく、動作試験には 1 が要ります
- **`control_mode` 4（位置指令）は CM4 が HEAD の機体だけ**です。旧 CM4 + G474 へ素通しすると暴走します
- `robot_id` は **0..10**。データグラムは 11 スロット固定で、11 番以上は符号化できません
- 読み直して組み直すと 2 バイト値が最大 1 LSB 落ちることがあります。`convertFloatToTwoByte` が
  丸めずに切り捨てるためで、C++ 側もまったく同じ挙動です（65536 通り中 189 通り）。
  発散はせず 3 パス以内・累積 2 LSB 以内で止まります
- フィードバックの byte 2 は **CRC ではありません**。G474 は定数 10 を書くだけなので検証に使いません
  （[robot_feedback_protocol.hpp](https://github.com/ibis-ssl/crane/blob/develop/crane_robot_receiver/include/crane_robot_receiver/robot_feedback_protocol.hpp)）。
  判定は同期バイトだけで行います

## レイアウトの正本

[robot_packet.h](https://github.com/ibis-ssl/crane/blob/develop/crane_sender/include/crane_sender/robot_packet.h) が正本です。
`crane_packet_forge/layout.py` はそこからの **生成物** で、手で編集しません。

```bash
python3 tools/gen_layout.py                                        # 再生成
ros2 run crane_packet_forge dump_golden > test/golden_vectors.json  # 基準ベクタ再生成
```

レンジは enum ではなく serialize 本体の `forward(..., range)` にしかありません。
名前から推測してはいけません — mode 3 の `target_global_velocity_theta` は名前が theta でも
**±M_PI ではなく ±32.767** です。

パケット仕様を変えるときは Crane / Orion_CM4 / G474 の同期と統合通信テストが要ります。
[ネットワークと実機通信](https://github.com/ibis-ssl/crane/blob/develop/docs/network.md#通信仕様を変更するとき)を参照してください。

## テスト

```bash
cd ~/ibis_ws && source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to crane_packet_forge
source install/local_setup.bash
colcon test --packages-select crane_packet_forge --event-handlers console_cohesion+
colcon test-result --verbose
```

- `test_layout_sync.py` — `layout.py` が `robot_packet.h` と一致するか（ヘッダが動いたら落ちる）
- `test_golden_vectors.cpp` — C++ のシリアライザが `golden_vectors.json` を再現するか
- `test_assemble_golden.py` — Python の組み立てが同じ JSON を再現するか（C++ と Python の一致）
- `test_roundtrip.py` — 不正な組み合わせを含む組み立て↔復号の性質（C++ に基準が無い領域）
- `test_sender_loop.py` — ループバックへ実際に送り、`check_counter` の連続性と終了時の停止指令を見る
- `test_cli.py` — CLI の入口。警告が出る経路も実行する
- `test_server_contract.py` — GUI が依存するサーバ側の約束事（空でも spec のキーを落とさない等）

ソースツリーから直接走らせることもできます（ROS 環境不要）:

```bash
cd crane_packet_forge && python3 -m pytest test/ -q
```
