uint8 id

geometry_msgs/Pose2D pose

geometry_msgs/Pose2D velocity
float32 velocity_norm

geometry_msgs/Pose2D acceleration
float32 acceleration_norm

# If robot is detected by vision in this frame, this field is set to true
bool detected

# Latest detection time
builtin_interfaces/Time last_vision_detection_stamp
builtin_interfaces/Time last_tracker_detection_stamp

#  Latest detected pose.
# This field is NOT filtered. This is a raw vision value.
geometry_msgs/Pose2D last_detection_pose

BallContact ball_contact
