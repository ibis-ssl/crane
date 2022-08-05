#
# crane_msgs/msg/RobotCommnad.msg
#
# from : crane_bt_executor
# to   : crane_real_sender
#

uint8 robot_id

#local goalie system(goalie only)
  bool local_goalie_enable false
#enable local control by FC sensor for centering ball in dribbler
  bool enable_ball_centering_control false

#for crane_local_planner
  bool disable_collision_avoidance false bool disable_goal_area_avoidance false bool
    disable_placement_avoidance false

#kick flag(chip / straight)
  bool chip_enable false

#kick power(0.0 - 1.0)
  float32 kick_power 0.0

#dribble power(0.0 - 1.0)
  float32 dribble_power 0.0

# target  [vx , vy , theta]
geometry_msgs/Pose2D target

# movement mode (transfer / motion)
bool motion_mode_enable

float32 current_theta 0.0
