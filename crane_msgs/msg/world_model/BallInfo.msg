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

# ボールモデルパラメータ
float32 deceleration    # 転がり時の減速度 (m/s²)
float32 gravity         # 重力加速度 (m/s²)
float32 air_resistance  # 空気抵抗係数 (将来使用)
