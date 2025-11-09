# 位置・速度
geometry_msgs/Vector3 position
geometry_msgs/Vector3 velocity

float32 velocity_norm

Vector3Stamped vision

bool detected

# ボール状態
uint8 STOPPED = 0
uint8 ROLLING = 1
uint8 FLYING = 2
uint8 state

# 物理モデル設定 (BallPhysicsModel対応)
BallPhysicsConfig physics_config
