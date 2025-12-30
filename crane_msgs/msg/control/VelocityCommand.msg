# VelocityCommand.msg
# local_planner → sender 用の速度指令メッセージ

uint8 robot_id

# 速度目標（極座標）
float32 target_velocity_r
float32 target_velocity_theta

# 将来の位置指令対応用（オプション）
float32[<=1] target_x
float32[<=1] target_y

# 角度目標
float32 target_theta
float32 omega_limit 10.0

# キック・ドリブル
bool chip_enable false
float32 kick_power 0.0
float32 dribble_power 0.0

# フラグ
bool stop_flag false
bool enable_local_feedback false

# 制限値（local_plannerで解決済み）
float32 max_velocity
float32 max_acceleration

# 現在状態
geometry_msgs/Pose2D current_pose
geometry_msgs/Pose2D current_velocity

# レイテンシ情報
float32 latency_ms 0.0
uint16 elapsed_time_ms_since_last_vision 0

# デバッグ・診断
NamedString[] state_factors
string planner_name
crane_msgs/DelayCheckpoints delay_checkpoints
crane_msgs/LocalPlannerConfig local_planner_config
