# crane
AI components for RoboCup Small Size League.

## Description
## System Requirements
## Software Requirements
## Usage
### run
ros2 run <package_name> <node_name>

source ~/ros2_ws/install/local_setup.bash

ros2 run  crane_real_sender real_sender_node　：realsender

ros2 run joy joy_node　＋　ros2 run consai2r2_teleop teleop_node　　：ジョイスティック操作（id：0になってるので注意）

### test
colcon test
colcon test-result --verbose

## Install
## License
## Author