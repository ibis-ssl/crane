#
# crane_msgs/msg/PlaySituation.msg
#
# from : crane_play_switcher
# to   : crane_field_analyzer
#
# for use in : RoleScores.msg
#

# ID : text
# 0  : HALT
# 1  : STOP
# 2  : ---
# 3  : FORCE_START
# 11 : OUR_KICKOFF_PREPARATION
# 12 : OUR_KICKOFF_START
# 13 : OUR_PENALTY_PREPARATION
# 14 : OUR_PENALTY_START
# 15 : OUR_DIRECT_FREE
# 16 : OUR_INDIRECT_FREE
# 17 : OUR_TIMEOUT
# 18 : OUR_GOAL
# 19 : OUR_BALL_PLACEMENT
# 20 : ---
# 21 : THEIR_KICKOFF_PREPARATION
# 22 : THEIR_KICKOFF_START
# 23 : THEIR_PENALTY_PREPARATION
# 24 : THEIR_PENALTY_START
# 25 : THEIR_DIRECT_FREE
# 26 : THEIR_INDIRECT_FREE
# 27 : THEIR_TIMEOUT
# 28 : THEIR_GOAL
# 29 : THEIR_BALL_PLACEMENT

# FIXME 多分本当はreferee系メッセージの大元で定義するべき
uint8 HALT = 0
uint8 STOP = 1
uint8 OUR_BALL_PLACEMENT = 19
uint8 THEIR_BALL_PLACEMENT = 29

uint8   referee_id
string  referee_text

bool    is_inplay
crane_msgs/InPlaySituation   inplay_situation

geometry_msgs/Point placement_position

crane_msgs/WorldModel world_model
