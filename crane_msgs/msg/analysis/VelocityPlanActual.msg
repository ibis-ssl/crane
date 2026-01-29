# 過去の計画と実際の比較データ

# 計画作成時刻（基準からの相対時間 us）
int32 plan_time_us

# 計画内容
float32 planned_vel_x
float32 planned_vel_y
float32 planned_pos_x
float32 planned_pos_y

# 実績（target_time_usに到達した時点）
float32 actual_vel_x
float32 actual_vel_y
float32 actual_pos_x
float32 actual_pos_y

# 誤差
float32 velocity_error  # |actual - planned|
float32 position_error  # 位置誤差[m]
