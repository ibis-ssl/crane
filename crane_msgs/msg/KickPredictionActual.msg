# キック予測と実際の比較データ

# 予測値
float32 predicted_ball_speed
float32 predicted_stop_distance

# 実績値
float32 actual_ball_speed     # 実際のボール初速度 [m/s]
float32 actual_stop_distance  # 実際の停止距離 [m]

# 誤差
float32 speed_error           # 速度誤差 [m/s]
float32 speed_error_percent   # 速度誤差率 [%]
float32 distance_error        # 距離誤差 [m]
float32 distance_error_percent # 距離誤差率 [%]
