std_msgs/Header header

bool on_positive_half
bool is_yellow
bool is_emplace_positive_side
bool half_court_practice_mode

uint8 our_goalie_id
uint8 their_goalie_id

uint32 our_max_allowed_bots
uint32 their_max_allowed_bots

FieldSize field_info
FieldSize penalty_area_size
FieldSize goal_size

BallInfo ball_info

RobotInfo[] robot_info_ours
RobotInfo[] robot_info_theirs

PlaySituation play_situation

GameAnalysis game_analysis

# 遅延監視用チェックポイント情報
# 各処理段階でのタイムスタンプを記録し、パフォーマンス分析とボトルネック特定を可能にする
crane_msgs/DelayCheckpoints delay_checkpoints
