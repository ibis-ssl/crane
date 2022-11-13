cmake_minimum_required(VERSION 3.5)
project(crane_teleop)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

#include_directories(include)

ament_auto_add_library(joystick_component SHARED src/joystick_component.cpp)
#target_compile_definitions(joystick_component PRIVATE "RASPIMOUSE_EXAMPLES_BUILDING_DLL")
#ament_target_dependencies(joystick_component
#  rclcpp
#  rclcpp_components
#  std_msgs
#  sensor_msgs
#  crane_msgs
#)
#rclcpp_components_register_nodes(joystick_component "joystick::JoystickComponent")

ament_auto_add_executable(teleop_node src/teleop_node.cpp)
#target_link_libraries(teleop_node joystick_component)
#ament_target_dependencies(
#  teleop_node
#  "rclcpp"
#)
#
#install(TARGETS
#  joystick_component
#  ARCHIVE DESTINATION lib
#  LIBRARY DESTINATION lib
#  RUNTIME DESTINATION bin)
#install(TARGETS teleop_node
#  EXPORT export_${PROJECT_NAME}
#  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
