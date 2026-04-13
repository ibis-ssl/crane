# 位置・速度
geometry_msgs/Vector3 position
geometry_msgs/Vector3 velocity

float32 velocity_norm

Vector3Stamped vision
Vector3Stamped tracker

bool detected

# ソース別検出フラグ（vision/trackerの使い分け用）
bool vision_detected
bool tracker_detected

# ボール状態
uint8 STOPPED = 0
uint8 ROLLING = 1
uint8 FLYING = 2
uint8 state

# 物理モデル設定 (BallPhysicsModel対応)
BallPhysicsConfig physics_config

# ボール予測トレース（予実管理用）
crane_msgs/BallPredictionTrace[<=1] ball_prediction_trace

# --- Fallback 推定情報（メイン値は書き換えない） ---
uint8 FALLBACK_NONE = 0
uint8 FALLBACK_LAST_KNOWN = 1
uint8 FALLBACK_BALL_SENSOR = 2
uint8 FALLBACK_OCCLUSION_SHADOW = 3
uint8 fallback_source

bool fallback_available
geometry_msgs/Vector3 fallback_position
float32 fallback_confidence
builtin_interfaces/Time fallback_stamp
uint32 fallback_occluder_robot_id
