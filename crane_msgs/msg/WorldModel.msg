#
# crane_msgs/msg/ControlTarget.msg
#
# from : crane_bt_executor
# to   : crane_local_planner
#

std_msgs/Header header

uint8 our_goalie_id
uint8 their_goalie_id

FieldSize field_info

DefenseArea our_defense
DefenseArea their_defense

BallInfo ball_info

RobotInfoOurs[] robot_info_ours
RobotInfoTheirs[] robot_info_theirs
