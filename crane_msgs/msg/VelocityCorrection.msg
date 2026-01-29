# 速度修正の詳細（RVO2以外にも汎用利用可能）

# 修正元
string source  # "rvo2", "sender_accel_limit", "feedback_control"

# 修正前の希望速度（m/s）
float32 before_vel_x
float32 before_vel_y

# 修正後の実際の速度（m/s）
float32 after_vel_x
float32 after_vel_y

# 速度変化量
float32 velocity_delta  # |after - before|
float32 direction_delta_deg  # 方向変化（度）
