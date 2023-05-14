cmake_minimum_required(VERSION 3.5)
project(crane_session_controller)

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

add_definitions("-DBOOST_ALLOW_DEPRECATED_HEADERS")

find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp_components REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME}_component SHARED
  src/crane_session_controller.cpp include/crane_session_controller/session_controller.hpp)
target_link_libraries(${PROJECT_NAME}_component ${YAML_CPP_LIBRARIES})

rclcpp_components_register_nodes(${PROJECT_NAME}_component "crane::SessionControllerComponent")

ament_auto_add_executable(${PROJECT_NAME}_node src/crane_session_controller_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_component)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(test_session_controller test/test.cpp)
  target_link_libraries(test_session_controller ${PROJECT_NAME}_component)
endif()

ament_auto_package(INSTALL_TO_SHARE config)
