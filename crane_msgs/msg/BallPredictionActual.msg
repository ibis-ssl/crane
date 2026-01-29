# 過去の予測と実際の比較データ

# 予測作成時刻（基準からの相対時間 us）
int32 prediction_time_us

# 予測内容
float32 predicted_pos_x
float32 predicted_pos_y
float32 predicted_pos_z
float32 predicted_vel_x
float32 predicted_vel_y

# 実績（target_time_usに到達した時点）
float32 actual_pos_x
float32 actual_pos_y
float32 actual_pos_z
float32 actual_vel_x
float32 actual_vel_y

# 誤差
float32 position_error  # 位置誤差[m]
float32 velocity_error  # 速度誤差[m/s]
uint8 actual_state      # 実際の状態（STOPPED/ROLLING/FLYING）
