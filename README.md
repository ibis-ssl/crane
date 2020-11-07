# crane
AI components for RoboCup Small Size League.

## Description
## System Requirements
## Software Requirements
## Usage
Before you execute, you should source workspace environment
~~~bash
source ~/ros2_ws/install/local_setup.bash
~~~
### Demo (rotation move & pass)
~~~bash
ros2 launch crane_bt_executor test.py
~~~
### Demo(Joystick)
~~~bash
ros2 run  crane_real_sender real_sender_node　：realsender
~~~
ジョイスティック操作（id：0になってるので注意）
~~~bash
ros2 run joy joy_node　＋　ros2 run consai2r2_teleop teleop_node
~~~
### test
colcon test
colcon test-result --verbose

## Install
## License
## Author
