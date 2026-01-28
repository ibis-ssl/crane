# PositionCommand.msg
# session_planner → local_planner 用の位置指令メッセージ

uint8 robot_id

# 位置目標
float32 target_x
float32 target_y
float32 position_tolerance 0.0
float32 speed_limit_at_target 0.0

# 角度目標
float32 target_theta
float32 omega_limit 10.0

# キック・ドリブル
bool chip_enable false
float32 kick_power 0.0
float32 dribble_power 0.0

# フラグ
bool local_goalie_enable false
bool enable_ball_centering_control false
bool stop_flag false

# LocalPlanner設定
crane_msgs/LocalPlannerConfig local_planner_config

# 現在状態（LocalPlannerで上書き）
geometry_msgs/Pose2D current_pose
geometry_msgs/Pose2D current_velocity

# デバッグ・診断
NamedString[] planning_factors
string planner_name
crane_msgs/DelayCheckpoints delay_checkpoints

# 速度計画トレース（session_plannerから伝播）
crane_msgs/VelocityPlanTrace[<=1] velocity_plan_trace
