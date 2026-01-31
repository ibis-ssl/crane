cmake_minimum_required(VERSION 3.12)
project(crane_sender)

# Default to C++20
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)

  # Debug symbols only for Debug builds
  if(CMAKE_BUILD_TYPE STREQUAL "Debug" OR CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
    add_compile_options(-g)
  endif()
endif ()

add_definitions("-DBOOST_ALLOW_DEPRECATED_HEADERS")

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp_components REQUIRED)
ament_auto_find_build_dependencies()

include_directories(
        include
        ${robocup_ssl_msgs_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
)

ament_auto_add_library(sim_sender_component SHARED
  src/sim_sender.cpp)

ament_auto_add_executable(sim_sender_node src/sim_sender_node.cpp)
ament_auto_add_executable(ibis_sender_node src/ibis_sender_node.cpp src/broadcast_command_sender.cpp)

add_library(crane_sender_base STATIC src/sender_base.cpp)
set_target_properties(crane_sender_base PROPERTIES POSITION_INDEPENDENT_CODE ON)
ament_target_dependencies(crane_sender_base
  rclcpp
  crane_msgs
  crane_msg_wrappers
  crane_comm
  crane_physics
)
target_include_directories(crane_sender_base PUBLIC include)

target_link_libraries(sim_sender_component crane_sender_base ${robocup_ssl_msgs_LIBRARIES})
target_link_libraries(sim_sender_node crane_sender_base)
target_link_libraries(ibis_sender_node crane_sender_base)

# Precompiled headers for faster compilation
target_precompile_headers(sim_sender_component PRIVATE
  <rclcpp/rclcpp.hpp>
  <crane_geometry/boost_geometry.hpp>
  <crane_geometry/geometry_operations.hpp>
  <robocup_ssl_msgs/ssl_simulation_robot_control.pb.h>
)

target_precompile_headers(crane_sender_base PRIVATE
  <rclcpp/rclcpp.hpp>
  <crane_msg_wrappers/world_model_wrapper.hpp>
)

if (BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_auto_add_gtest(test_robot_packet test/test_robot_packet.cpp)
  target_link_libraries(test_robot_packet sim_sender_component)
  target_link_libraries(test_robot_packet crane_sender_base)
endif ()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
