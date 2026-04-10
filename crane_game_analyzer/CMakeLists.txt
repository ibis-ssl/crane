cmake_minimum_required(VERSION 3.12)
project(crane_game_analyzer)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  if(NOT CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_options(-g)
  endif()
endif()

add_definitions("-DBOOST_ALLOW_DEPRECATED_HEADERS")

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp_components REQUIRED)
ament_auto_find_build_dependencies()

message("closest_point_vendor_DIR")
message(${closest_point_vendor_DIR})
ament_auto_add_library(${PROJECT_NAME}_component SHARED
  src/crane_game_analyzer.cpp
  src/evaluations.cpp
  src/kick_event_detector.cpp
  src/threat_evaluator.cpp
  src/metrics/metric_engine.cpp
  src/metrics/threat_metrics.cpp
  src/metrics/ball_horizon_metric.cpp
  src/metrics/ongoing_kick_metric.cpp
  src/metrics/slack_metrics.cpp
  src/metrics/attacker_metrics.cpp
  src/metrics/sub_attacker_metrics.cpp
  src/metrics/pass_target_metrics.cpp
)

option(CRANE_UNITY_BUILD "Enable unity build" ON)
if(CRANE_UNITY_BUILD)
  set_target_properties(${PROJECT_NAME}_component PROPERTIES UNITY_BUILD ON)
  set_target_properties(${PROJECT_NAME}_component PROPERTIES UNITY_BUILD_BATCH_SIZE 6)
endif()

ament_auto_add_executable(${PROJECT_NAME}_node
  src/${PROJECT_NAME}_node.cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
