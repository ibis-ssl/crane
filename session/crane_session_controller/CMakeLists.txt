cmake_minimum_required(VERSION 3.5)
project(crane_session_controller)

find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp_components REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME}_component SHARED
        src/crane_session_controller.cpp include/crane_session_controller/session_controller.hpp)

rclcpp_components_register_nodes(${PROJECT_NAME}_component "crane::SessionContollerComponent")

ament_auto_add_executable(${PROJECT_NAME}_node src/crane_session_controller_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_component)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
