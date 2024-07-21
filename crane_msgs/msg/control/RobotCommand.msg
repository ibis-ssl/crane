uint8 robot_id

# local goalie system(goalie only)
bool local_goalie_enable false
# enable local control by FC sensor for centering ball in dribbler
bool enable_ball_centering_control false

crane_msgs/LocalPlannerConfig local_planner_config

# kick flag(chip / straight)
bool chip_enable false

# emergency stop flag (stop wheel motors immediately)
bool stop_flag false

# if this flag is true, the robot will lift up dribbler (but chip_enable has priority)
bool lift_up_dribbler_flag false

# kick power(0.0 - 1.0)
float32 kick_power 0.0

# dribble power(0.0 - 1.0)
float32 dribble_power 0.0

# target[vx, vy, theta]
geometry_msgs/Pose2D target_velocity

geometry_msgs/Pose2D current_velocity

geometry_msgs/Pose2D current_pose

bool enable_local_feedback

float32[] target_x
float32[] target_y
float32[] target_theta

float32 latency_ms 0.0

float32 current_ball_x
float32 current_ball_y

string skill_name

# movement mode(transfer / motion)
bool motion_mode_enable
