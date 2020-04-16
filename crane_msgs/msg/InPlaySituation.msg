#
# crane_msgs/msg/InPlaySituation.msg
#
# for use in : PlaySituation.msg
#

bool ball_possession_ours # when both team scramble, both flags will be true
bool ball_possession_theirs # when both team scramble, both flags will be true
uint8 nearest_to_ball_robot_id_ours
uint8 nearest_to_ball_robot_id_theirs
