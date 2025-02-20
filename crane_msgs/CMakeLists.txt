cmake_minimum_required(VERSION 3.5)
project(crane_msgs)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(msg_files
  "msg/world_model/BallInfo.msg"
  "msg/world_model/BallContact.msg"
  "msg/world_model/PenaltyArea.msg"
  "msg/world_model/FieldSize.msg"
  "msg/world_model/RobotInfo.msg"
  "msg/world_model/WorldModel.msg"

  "msg/analysis/BallAnalysis.msg"
  "msg/analysis/GameAnalysis.msg"
  "msg/analysis/PingStatus.msg"
  "msg/analysis/PingStatusArray.msg"
  "msg/analysis/PlaySituation.msg"
  "msg/analysis/FieldAnalysis.msg"
  "msg/analysis/NamedBool.msg"
  "msg/analysis/NamedFloat.msg"
  "msg/analysis/NamedInt.msg"
  "msg/analysis/NamedPosition.msg"
  "msg/analysis/NamedString.msg"
  "msg/analysis/NamedValueArray.msg"
  "msg/analysis/Kick.msg"

  "msg/planner/PassPlan.msg"
  "msg/planner/PassInfo.msg"
  "msg/planner/ReceiverPlan.msg"

  "msg/control/LocalPlannerConfig.msg"
  "msg/control/StateFactor.msg"
  "msg/control/RobotCommand.msg"
  "msg/control/RobotCommands.msg"
  "msg/control/RobotFeedback.msg"
  "msg/control/RobotFeedbackArray.msg"

  "msg/control/LocalCameraMode.msg"
  "msg/control/PositionTargetMode.msg"
  "msg/control/SimpleVelocityTargetMode.msg"
  "msg/control/PolarVelocityTargetMode.msg"
)

set(srv_files
  "srv/PassRequest.srv"
  "srv/RobotSelect.srv"
)

set(action_files
  "action/SkillExecution.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs robocup_ssl_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
