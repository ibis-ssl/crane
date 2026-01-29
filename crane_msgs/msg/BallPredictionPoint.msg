# ボール予測の予測点
# 「N秒後にボールがどこにあるか」を表現

# 予測作成元（どのコンポーネントで作成されたか）
string source  # "pass_selector", "goalie", "planner", "world_model"

# 予測対象時刻（基準からの相対時間 us）
int32 target_time_us

# 予測位置（フィールド座標 m）
float32 predicted_pos_x
float32 predicted_pos_y
float32 predicted_pos_z  # 高度（飛行時）

# 予測速度（フィールド座標 m/s）
float32 predicted_vel_x
float32 predicted_vel_y
float32 predicted_vel_z

# 予測されたボール状態
uint8 STOPPED = 0
uint8 ROLLING = 1
uint8 FLYING = 2
uint8 predicted_state
