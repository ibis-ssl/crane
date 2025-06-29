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
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME}_component SHARED
  src/world_model_publisher.cpp
  src/world_model_data_provider.cpp
  src/vision_data_processor.cpp
  src/tracker_data_processor.cpp
  src/ball_tracker.cpp
  src/ball_physics_model.cpp
  src/robot_tracker.cpp
  src/data_source_manager.cpp
  src/data_source_adapters.cpp
  src/vision_packet_receiver.cpp
  src/vision_data_converter.cpp
  src/tracker_manager_factory.cpp
  src/visualization_manager.cpp
)

# Enable Unity Build for faster compilation
set_target_properties(${PROJECT_NAME}_component PROPERTIES UNITY_BUILD ON)
set_target_properties(${PROJECT_NAME}_component PROPERTIES UNITY_BUILD_BATCH_SIZE 4)

target_link_libraries(${PROJECT_NAME}_component
        ${robocup_ssl_msgs_LIBRARIES} ${Boost_LIBRARIES}
)

target_precompile_headers(${PROJECT_NAME}_component
  PUBLIC
    "<crane_geometry/boost_geometry.hpp>"
    "<crane_geometry/geometry_operations.hpp>"
    "<crane_comm/time.hpp>"
    "<crane_comm/ddps.hpp>"
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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
