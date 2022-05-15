cmake_minimum_required(VERSION 3.5)
project(crane_joy_simulator)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include ${CMAKE_CURRENT_BINARY_DIR})
add_executable(joy_simulator_node
  src/joy_simulator_node.cpp
)
ament_target_dependencies(
  joy_simulator_node
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
)

install(TARGETS joy_simulator_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
