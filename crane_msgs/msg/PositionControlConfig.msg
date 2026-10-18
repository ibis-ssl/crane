# CM4 側の位置制御 PID へ crane が送った設定そのもの。
#
# フィールドは Orion_CM4 cm4/bridge/config_packet.h の 28 バイトパケットと 1:1 対応する。
# 設定パケットを送るたびに publish するので、bag にはゲイン変更の時刻が残る。
# 記録する理由と読み方は docs/network.md を参照。

std_msgs/Header header

# 全機に適用する値であることを示す target_robot_id
uint8 TARGET_ALL_ROBOTS=255

# パケット byte 4: フォーマット版（現在は 2 = PID 対応版）
uint8 packet_version
# パケット byte 5: 適用先ロボット。255 なら全機に適用（UDP の宛先は機体ごとに指定する）
uint8 target_robot_id

# パケット byte 8..27: この順序で float32 が 5 つ並ぶ
float32 kp
float32 deceleration
float32 tolerance
float32 ki
float32 kd

# 直近に指令を送った機体すべての宛先について、UDP ソケットへ値を渡せたか。
# 送る機体が無かったときは false。CM4 は ACK を返さないので true でも受信・受理の
# 証拠にはならない（範囲外の値はデータグラムごと CM4 に破棄される）。
# 送信に失敗した場合も値は残す（黙って消えると原因追跡ができない）
bool sent
