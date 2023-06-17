cmake_minimum_required(VERSION 3.5)
project(crane_robot_receiver)

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

include_directories(include $crane_lint_commonCMAKE_CURRENT_BINARY_DIR})
add_executable(robot_receiver_node
        src/robot_receiver_node.cpp
)

ament_target_dependencies(
  robot_receiver_node
  "rclcpp"
  "std_msgs"
)

install(TARGETS robot_receiver_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
