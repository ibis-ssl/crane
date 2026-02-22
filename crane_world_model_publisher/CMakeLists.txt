cmake_minimum_required(VERSION 3.12)
project(crane_world_model_publisher)

# Default to C++20
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  if(NOT CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_options(-g)
  endif()
endif ()

add_definitions("-DBOOST_ALLOW_DEPRECATED_HEADERS")

find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(yaml-cpp REQUIRED)
ament_auto_find_build_dependencies()

include_directories(${robocup_ssl_msgs_INCLUDE_DIRS})

option(CRANE_WMP_BUILD_CALIBRATION "Build optional calibration utilities" OFF)

set(WORLD_MODEL_PUBLISHER_SRCS
  src/world_model_publisher.cpp
  src/world_model_data_provider.cpp
  src/visualization_manager.cpp
)

if(CRANE_WMP_BUILD_CALIBRATION)
  list(APPEND WORLD_MODEL_PUBLISHER_SRCS
    src/calibration/ball_calibration_data_extractor.cpp
    src/calibration/simple_ball_physics_optimizer.cpp
  )
endif()

ament_auto_add_library(${PROJECT_NAME}_component SHARED ${WORLD_MODEL_PUBLISHER_SRCS})

# Enable Unity Build for faster compilation
option(CRANE_UNITY_BUILD "Enable unity build" ON)
if(CRANE_UNITY_BUILD)
  set_target_properties(${PROJECT_NAME}_component PROPERTIES UNITY_BUILD ON)
  set_target_properties(${PROJECT_NAME}_component PROPERTIES UNITY_BUILD_BATCH_SIZE 4)
endif()

target_link_libraries(${PROJECT_NAME}_component
        ${robocup_ssl_msgs_LIBRARIES}
        robocup_ssl_msgs::robocup_ssl_msgs_proto
        ${Boost_LIBRARIES}
        yaml-cpp
)

rclcpp_components_register_nodes(${PROJECT_NAME}_component "crane::WorldModelPublisherComponent")

ament_auto_add_executable(${PROJECT_NAME}_node src/world_model_publisher_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_component glog)

if(CRANE_WMP_BUILD_CALIBRATION)
  ament_auto_add_executable(ball_calibration_node src/calibration/ball_calibration_node.cpp)
  target_link_libraries(ball_calibration_node ${PROJECT_NAME}_component glog)
endif()

# Pythonスクリプトのインストール
install(PROGRAMS
  scripts/plot_kick_events.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR INSTALL_TO_SHARE launch config)
