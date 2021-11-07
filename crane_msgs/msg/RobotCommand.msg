#
# crane_msgs/msg/RobotCommnad.msg
#
# from : crane_bt_executor
# to   : crane_real_sender
#

uint8 robot_id

# local goalie system(goalie only)
bool local_goalie_enable false

# collision avoidance
bool disable_collision_avoidance  false
# bool disble_ball_avoidance = false;
# bool disable_agent_avoidance = false
# bool diable_goal_area_avoidance = false
# bool disable_external_area_avoidance = false


# kick flag (chip/straight)
bool chip_enable false

# kick power (0.0 - 1.0)
float32 kick_power 0.0

# dribble power (0.0 - 1.0)
float32 dribble_power 0.0

# target  [vx , vy , theta]
geometry_msgs/Pose2D target

float32 current_theta 0.0