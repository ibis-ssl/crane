uint8 id

geometry_msgs/Pose2D pose

geometry_msgs/Pose2D velocity
float32 velocity_norm

geometry_msgs/Pose2D acceleration
float32 acceleration_norm

# If robot is detected by vision in this frame, this field is set to true

Pose2DStamped vision

# 入力ソース別の検出状態
bool available_vision                # ビジョンで検出されているか
bool available_feedback              # フィードバックで検出されているか
bool available_tracker               # トラッカーで検出されているか

# Latest detection time
builtin_interfaces/Time last_tracker_detection_stamp
builtin_interfaces/Time last_feedback_detection_stamp


BallContact ball_contact

builtin_interfaces/Time last_ball_sensor_stamp
bool ball_sensor

# ロボットのエラー情報（/robot_feedback 由来）
# id または info が非ゼロであればエラー有りとみなす
bool has_error
uint16 error_id
uint16 error_info
float32 error_value
builtin_interfaces/Time last_error_stamp
float32 error_duration_sec

# モーター温度情報（/robot_feedback 由来）
# motor_temperatures[0-3]: モーター1-4の温度（℃）
uint8[] motor_temperatures
