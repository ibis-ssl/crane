#
# crane_msgs / msg / RobotCommnad.msg
#
# from : crane_bt_executor
# to : crane_real_sender
#

uint8 robot_id

# local goalie system(goalie only)
bool local_goalie_enable false
# enable local control by FC sensor for centering ball in dribbler
bool enable_ball_centering_control false

crane_msgs/LocalPlannerConfig local_planner_config

# kick flag(chip / straight)
bool chip_enable false

# kick power(0.0 - 1.0)
float32 kick_power 0.0

# dribble power(0.0 - 1.0)
float32 dribble_power 0.0

# target[vx, vy, theta]
geometry_msgs/Pose2D target_velocity

geometry_msgs/Pose2D current_pose

float32[] target_x
float32[] target_y
float32[] target_theta

float32 laytency_ms 0.0

float32 ball_relative_position_x
float32 ball_relative_position_y
float32 ball_relative_velocity_x
float32 ball_relative_velocity_y


# movement mode(transfer / motion)
bool motion_mode_enable
