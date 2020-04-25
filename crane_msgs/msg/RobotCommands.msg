#
# crane_msgs/msg/RobotCommands.msg
#
# from : crane_bt_executor
# to   : consai2r2_sender
#

std_msgs/Header header
bool is_yellow
RobotCommand[] robot_commands
