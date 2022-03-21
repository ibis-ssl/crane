#
# crane_msgs/msg/BallInfo.msg
#
# from : consai2r2/vision_wrapper (not exist)
# to   : crane_world_observer
#

# ボールが消えたかどうか
bool disappeared
# カーブボールかどうか
bool curved
# 位置・速度
geometry_msgs/Pose2D pose
geometry_msgs/Pose2D velocity

bool detected
float64 detection_time
