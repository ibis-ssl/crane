#
# crane_msgs/msg/RoleCommands.msg
#
# from : crane_reveive_planner
# to   : crane_target_area_planner
#
crane_msgs/PassPlan pass_plan
# pose to receive the ball
geometry_msgs/Point receive_position
std_msgs/Float32 receive_theta
