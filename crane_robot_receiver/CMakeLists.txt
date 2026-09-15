cmake_minimum_required(VERSION 3.12)
project(crane_robot_receiver)

# 既定を Release にする。CMAKE_BUILD_TYPE 未指定だと GCC は最適化なし（-O0）になり、
# 経路計画のような毎周期の計算が桁違いに遅くなる。CI と docker は明示的に Release を
# 指定しているので、これはローカルビルドを同じ条件に揃えるためのもの。
# デバッグ時は -DCMAKE_BUILD_TYPE=Debug または RelWithDebInfo を明示する。
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  if(NOT CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_options(-g)
  endif()
endif()

add_definitions("-DBOOST_ALLOW_DEPRECATED_HEADERS")

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(EXISTS "/opt/ros/jazzy/share/backward_ros/cmake/BackwardConfig.cmake")
  include(/opt/ros/jazzy/share/backward_ros/cmake/BackwardConfig.cmake)
else()
  macro(add_backward target)
  endmacro()
endif()

include_directories(include ${CMAKE_CURRENT_BINARY_DIR} ${robocup_ssl_msgs_INCLUDE_DIRS})
ament_auto_add_executable(robot_receiver_node
  src/robot_receiver_node.cpp
)
add_backward(robot_receiver_node)
target_link_libraries(robot_receiver_node atomic)

ament_auto_add_executable(ping_status_node
        src/ping_status_node.cpp
)

ament_auto_add_executable(diagnostic_publisher_node
        src/diagnostic_publisher.cpp
        src/diagnostic_publisher_node.cpp
)
target_link_libraries(diagnostic_publisher_node fmt)
add_backward(diagnostic_publisher_node)

ament_auto_add_executable(grsim_robot_status_node
        src/grsim_robot_status_node.cpp
)
# これがないと参照未解決エラーが出る
target_link_libraries(grsim_robot_status_node
        ${robocup_ssl_msgs_LIBRARIES}
        robocup_ssl_msgs::robocup_ssl_msgs_proto)
add_backward(grsim_robot_status_node)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(
    test_robot_feedback_protocol
    test/test_robot_feedback_protocol.cpp
  )
  target_include_directories(test_robot_feedback_protocol PRIVATE include)
  ament_target_dependencies(test_robot_feedback_protocol rclcpp)
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
