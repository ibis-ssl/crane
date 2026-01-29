# 速度計画の予測点
# 「N秒後にMm先で速度Lm/sに達する予定」を表現

# 計画作成元（どのレイヤーで作成されたか）
string source  # "skill", "session", "local_planner", "sender"

# 予測対象時刻（基準からの相対時間 us）
int32 target_time_us

# 予測位置（フィールド座標 m）
float32 predicted_pos_x
float32 predicted_pos_y

# 予測速度（フィールド座標 m/s）
float32 predicted_vel_x
float32 predicted_vel_y

# 目標到達予定時刻（us）
int32 estimated_arrival_time_us
