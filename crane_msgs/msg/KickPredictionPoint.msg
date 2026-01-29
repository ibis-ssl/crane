# キック予測の予測点

# 予測作成元
string source  # "skill", "session", "kick_event_detector"

# キック情報
float32 kick_power           # キック力 [0.0-1.0]
bool is_chip_kick            # チップキックか

# 予測値
float32 predicted_ball_speed # 予測ボール初速度 [m/s]
float32 predicted_stop_distance # 予測停止距離 [m]

# キック時のボール位置
float32 kick_pos_x
float32 kick_pos_y

# タイムスタンプ（キック時刻、us）
int64 kick_timestamp_us
