cmake_minimum_required(VERSION 3.5)
project(crane_dummy_pass_planner)

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
find_package(crane_msgs REQUIRED)
find_package(crane_msg_wrappers REQUIRED)

include_directories(include ${CMAKE_CURRENT_BINARY_DIR})
add_executable(${PROJECT_NAME}_node
  src/${PROJECT_NAME}_node.cpp
)
ament_target_dependencies(${PROJECT_NAME}_node
  "rclcpp"
  "std_msgs"
  "crane_msgs"
  "crane_msg_wrappers"
)

install(TARGETS ${PROJECT_NAME}_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
