cmake_minimum_required(VERSION 3.5)
project(crane_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(consai2r2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(msg_files
    "msg/BallInfo.msg"
    "msg/DefenseArea.msg"
    "msg/FieldSize.msg"
    "msg/InPlaySituation.msg"
    "msg/PlaySituation.msg"
    "msg/RobotInfoOurs.msg"
    "msg/RobotInfoTheirs.msg"
    "msg/WorldModel.msg"
    "msg/BehaviorTreeCommand.msg"
    "msg/RoleScore.msg"
    "msg/RoleScores.msg"
    "msg/RoleCommand.msg"
    "msg/RoleCommands.msg"
    "msg/ControlTarget.msg"
    "msg/RobotCommand.msg"
    "msg/RobotCommands.msg"
    "msg/SubRole.msg"
    "msg/FieldAnalysis.msg"

)

set(srv_files
  
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs consai2r2_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
