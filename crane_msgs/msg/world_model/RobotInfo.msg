uint8 id

geometry_msgs/Pose2D pose

geometry_msgs/Pose2D velocity
float32 velocity_norm

geometry_msgs/Pose2D acceleration
float32 acceleration_norm

# If robot is detected by vision in this frame, this field is set to true

Pose2DStamped vision

bool vision_detected
bool feedback_detected
bool internal_tracker_detected
bool detected

# Latest detection time
builtin_interfaces/Time last_tracker_detection_stamp
builtin_interfaces/Time last_feedback_detection_stamp


BallContact ball_contact

builtin_interfaces/Time last_ball_sensor_stamp
bool ball_sensor
