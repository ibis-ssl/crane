cmake_minimum_required(VERSION 3.5)
project(crane_grsim_operator)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

#include_directories(include ${CMAKE_CURRENT_BINARY_DIR})
ament_auto_add_executable(${PROJECT_NAME}_node
  src/${PROJECT_NAME}_node.cpp
)
#ament_target_dependencies(${PROJECT_NAME}_node
#  "rclcpp"
#  "std_msgs"
#  "crane_msgs"
#  "robocup_ssl_msgs"
#)
#
#install(TARGETS ${PROJECT_NAME}_node
#  EXPORT export_${PROJECT_NAME}
#  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
