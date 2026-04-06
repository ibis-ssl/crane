int8 kicker_id -1
bool is_kicker_friend

float32 origin_x
float32 origin_y

float32 direction

# キックコマンド情報（味方ロボットの場合のみ）
float32 commanded_kick_power 0.0  # コマンドで指定されたキック力 [0.0-1.0]
bool commanded_chip_kick false    # コマンドで指定されたチップキックフラグ

# キック予測トレース
crane_msgs/KickPredictionTrace[<=1] kick_prediction_trace
