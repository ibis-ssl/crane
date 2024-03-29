cmake_minimum_required(VERSION 3.5)
project(crane_sender)

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif ()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp_components REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(sim_sender_component SHARED
  src/sim_sender.cpp)

ament_auto_add_executable(sim_sender_node src/sim_sender_node.cpp)
ament_auto_add_executable(ibis_sender_node src/ibis_sender_node.cpp)
add_backward(sim_sender_node)
add_backward(ibis_sender_node)

if (BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(test_robot_packet test/test_robot_packet.cpp)
  target_link_libraries(test_robot_packet sim_sender_component)
endif ()

ament_auto_package(INSTALL_TO_SHARE launch config)
