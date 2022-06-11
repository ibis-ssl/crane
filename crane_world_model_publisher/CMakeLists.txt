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
find_package(rclcpp_components REQUIRED)
find_package(robocup_ssl_msgs REQUIRED)
find_package(crane_msgs REQUIRED)
find_package(crane_geometry REQUIRED)

include_directories(include ${CMAKE_CURRENT_BINARY_DIR})

add_library(world_model_publisher_component SHARED
        src/world_model_publisher.cpp include/crane_world_model_publisher/world_model_publisher.hpp)


target_compile_definitions(world_model_publisher_component
PRIVATE "CRANE_BUILDING_DLL")

rclcpp_components_register_nodes(world_model_publisher_component "crane::WorldModelPublisherComponent")

add_executable(world_model_publisher_node
  src/world_model_publisher_node.cpp
)

target_link_libraries(world_model_publisher_node world_model_publisher_component)

ament_target_dependencies(world_model_publisher_component
  "rclcpp"
  "rclcpp_components"
  "robocup_ssl_msgs"
  "crane_msgs"
  "crane_geometry"
)

# Exports
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(rclcpp)
ament_export_dependencies(rclcpp_components)
ament_export_dependencies(robocup_ssl_msgs)
ament_export_dependencies(crane_msgs)
ament_export_dependencies(crane_geometry)

ament_export_include_directories(include)
ament_export_libraries(world_model_publisher_component)

install(TARGETS world_model_publisher_component
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

ament_package()
