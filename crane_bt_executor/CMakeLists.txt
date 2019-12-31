cmake_minimum_required(VERSION 3.5)
project(crane_bt_executor)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(crane_msgs REQUIRED)
find_package(crane_3rdparty REQUIRED)
find_package(behaviortree_cpp_v3_ibis REQUIRED)

 include_directories(include ${CMAKE_CURRENT_BINARY_DIR})
#include_directories(${CMAKE_CURRENT_BINARY_DIR})
add_executable(crane_bt_executor_node
  src/bt_executor_node.cpp
)
ament_target_dependencies(
  crane_bt_executor_node
  "rclcpp"
  "std_msgs"
  "crane_msgs"
  "behaviortree_cpp_v3_ibis"
  "crane_3rdparty"
)

# install(
#   DIRECTORY include/
#   DESTINATION include)

install(TARGETS crane_bt_executor_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
