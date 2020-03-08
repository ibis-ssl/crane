#
# crane_msgs/msg/RobotCommnad.msg
#
# from : crane_bt_executor
# to   : crane_real_sender
#

uint8 robot_id

# local goalie system(goalie only)
bool local_goalie_enable

# kick flag (chip/straight)
bool chip_enable

# kick power (0.0 - 1.0)
float32 kick_power

# dribble power (0.0 - 1.0)
float32 dribble_power

# target  [vx , vy , theta]
geometry_msgs/Pose2D target

float32 current_theta