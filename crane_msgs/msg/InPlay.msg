#
# crane_msgs/msg/InPlay.msg
#
# from : crane_play_switcher
# to   : crane_field_analyzer
#

bool ball_possession_ours # when both team scramble, both flags will be true
bool ball_possession_theirs # when both team scramble, both flags will be true
uint8 nearest_to_ball_robot_id_ours
uint8 nearest_to_ball_robot_id_theirs
