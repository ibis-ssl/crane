# RONAR (RObot Non-verbal Action Recognition) Event Message
# SSL実況システム用のイベント検出結果

std_msgs/Header header

# イベントタイプ定数
uint8 EVENT_POSSESSION_CHANGE = 0
uint8 EVENT_PASS = 1
uint8 EVENT_SHOT = 2
uint8 EVENT_FAST_SHOT = 3
uint8 EVENT_GOAL = 4
uint8 EVENT_SAVE = 5
uint8 EVENT_INTERCEPTION = 6
uint8 EVENT_BALL_OUT = 7
uint8 EVENT_SET_PLAY = 8
uint8 EVENT_COLLISION = 9

# Play Switcher イベント
uint8 EVENT_HALT = 10
uint8 EVENT_STOP = 11
uint8 EVENT_INPLAY_START = 12
uint8 EVENT_TIMEOUT = 13
uint8 EVENT_HALF_TIME = 14
uint8 EVENT_GAME_END = 15

# Autoref イベント
uint8 EVENT_FOUL = 20

# イベントタイプ
uint8 event_type

# イベント発生位置（フィールド座標）
geometry_msgs/Point position

# ボール速度 [m/s]
float32 ball_speed

# 検出信頼度 (0.0-1.0)
float32 confidence

# 主要ロボット（キッカー、シューター等）
bool has_primary_robot
uint8 primary_robot_id
bool primary_robot_is_ours

# 副次ロボット（パス受け手、ゴーリー等）
bool has_secondary_robot
uint8 secondary_robot_id
bool secondary_robot_is_ours

# 追加メタデータ（JSON形式）
# 例: {"pass_distance": 2.5, "shot_angle": 15.3}
string metadata_json
