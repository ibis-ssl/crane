# robot information for consai2_game

# robot id
uint8                robot_id

# Robot 2D pose
# This field will be replaced to geometry_msgs/Pose 
geometry_msgs/Pose2D pose

# Robot Velocity
# [Deprecated] This field will be removed. 
geometry_msgs/Pose2D velocity

# Robot Velocity
geometry_msgs/Twist  velocity_twist

# If robot is detected by vision in this frame, this field is set to true
bool                 detected

# Latest vision detection time
# [Deprecated] This field will be removed. 
builtin_interfaces/Time detection_stamp

#  Latest detected pose.
# This field is NOT filtered. This is a raw vision value.
geometry_msgs/Pose2D last_detection_pose

# If a robot disappear from field,  this field is set to true
bool                 disappeared
