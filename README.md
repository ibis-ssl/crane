# crane
AI components for RoboCup Small Size League.

## Description
## System Requirements
## Software Requirements

- Ubuntu 20.04
- ROS Galactic
- GitHubへSSH鍵を登録
  - https://hansrobo.github.io/mycheatsheets/git

~~~bash
mkdir -p ibis_ws/src
cd ibis_ws/src
git clone git@github.com:ibis-ssl/crane.git
source /opt/ros/galactic/setup.bash
rosdep install -riy --from-paths src
colcon build --symlink-install
source install/local_setup.bash
~~~

## Demo 
1. launch grsim
2. launch ros2 nodes
~~~bash
ros2 launch crane_bringup wait_planner.launch.py
ros2 run crane_wait_planner crane_wait_planner_node
ros2 run crane_goalie_planner crane_goalie_planner_node
ros2 run crane_defender_planner crane_defender_planner_node
~~~

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
