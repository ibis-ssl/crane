#
# crane_msgs/msg/ControlTarget.msg
#
# from : crane_bt_executor
# to   : crane_local_planner
#

std_msgs/Header header

bool is_yellow

uint8 our_goalie_id
uint8 their_goalie_id

FieldSize field_info
FieldSize defense_area_size
FieldSize goal_size

BallInfo ball_info

RobotInfoOurs[] robot_info_ours
RobotInfoTheirs[] robot_info_theirs
