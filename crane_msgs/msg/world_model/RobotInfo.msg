# robot information for consai2_game

# robot id
uint8                robot_id

# Robot 2D pose
geometry_msgs/Pose2D pose

# Robot Velocity
geometry_msgs/Pose2D velocity

# If robot is detected by vision in this frame, this field is set to true
bool                 detected

# Latest vision detection time
builtin_interfaces/Time detection_stamp

#  Latest detected pose.
# This field is NOT filtered. This is a raw vision value.
geometry_msgs/Pose2D last_detection_pose

# If a robot disappear from field,  this field is set to true
bool                 disappeared

BallContact ball_contact
