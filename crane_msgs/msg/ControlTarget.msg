#
# crane_msgs/msg/ControlTarget.msg
#
# from : crane_bt_executor
# to   : crane_local_planner
#

uint8 robot_id

# kick flag (chip/straight)
bool chip_kick_enable
bool straight_kick_enable

# kick power (0.0 - 1.0)
float32 kick_power

# dribble power (0.0 - 1.0)
float32 dribble_power

# movement mode (transfer / motion)
bool motion_mode_enable

# goal
# in motion mode , goal is  [vx , vy , theta]
# in transfer mode , goal is [x , y , theta]
geometry_msgs/Pose2D goal
