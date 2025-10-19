cmake_minimum_required(VERSION 3.5)
project(crane_world_model_publisher)

# Default to C++20
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif ()

add_definitions("-DBOOST_ALLOW_DEPRECATED_HEADERS")

find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(yaml-cpp REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME}_component SHARED
  src/world_model_publisher.cpp
  src/world_model_data_provider.cpp
  src/visualization_manager.cpp
  src/kick_event_detector.cpp
  src/pass_target_selector.cpp
  src/calibration/ball_calibration_data_extractor.cpp
  src/calibration/simple_ball_physics_optimizer.cpp
)

# Enable Unity Build for faster compilation
set_target_properties(${PROJECT_NAME}_component PROPERTIES UNITY_BUILD ON)
set_target_properties(${PROJECT_NAME}_component PROPERTIES UNITY_BUILD_BATCH_SIZE 4)

target_link_libraries(${PROJECT_NAME}_component
        ${robocup_ssl_msgs_LIBRARIES} ${Boost_LIBRARIES} yaml-cpp
)

target_precompile_headers(${PROJECT_NAME}_component
  PUBLIC
    "<crane_geometry/boost_geometry.hpp>"
    "<crane_geometry/geometry_operations.hpp>"
    "<crane_geometry/ddps.hpp>"
    "<crane_utils/time.hpp>"
    "<crane_msg_wrappers/world_model_wrapper.hpp>"
    "<crane_msgs/msg/world_model.hpp>"
    "<robocup_ssl_msgs/msg/referee.hpp>"
    "<rclcpp/rclcpp.hpp>"
    "<tf2_ros/transform_broadcaster.h>"
    "<tf2_geometry_msgs/tf2_geometry_msgs.hpp>"
)

rclcpp_components_register_nodes(${PROJECT_NAME}_component "crane::WorldModelPublisherComponent")

ament_auto_add_executable(${PROJECT_NAME}_node src/world_model_publisher_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_component glog)

ament_auto_add_executable(ball_calibration_node src/calibration/ball_calibration_node.cpp)
target_link_libraries(ball_calibration_node ${PROJECT_NAME}_component glog)

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
