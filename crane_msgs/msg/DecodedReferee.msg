# game status for consai2_game

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

uint8   referee_id
string  referee_text

bool    can_move_robot
float64 speed_limit_of_robot # meter/sec
bool    can_kick_ball
bool    can_enter_their_side
bool    can_enter_center_circle
bool    is_inplay
float64 keep_out_radius_from_ball # meter
float64 keep_out_distance_from_their_defense_area # meter

geometry_msgs/Point placement_position
