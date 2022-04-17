cmake_minimum_required(VERSION 3.5)
project(crane_world_model_publisher)

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
find_package(robocup_ssl_msgs REQUIRED)
find_package(crane_msgs REQUIRED)
find_package(crane_geometry REQUIRED)

include_directories(include ${CMAKE_CURRENT_BINARY_DIR})

add_executable(world_model_publisher
  src/world_model_publisher_node.cpp
)


ament_target_dependencies(
  world_model_publisher
  "rclcpp"
  "robocup_ssl_msgs"
  "crane_msgs"
  "crane_geometry"
)

install(TARGETS world_model_publisher
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY "include/"
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_package()
