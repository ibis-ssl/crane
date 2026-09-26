cmake_minimum_required(VERSION 3.12)
project(robocup_ssl_comm)

# 既定を Release にする。CMAKE_BUILD_TYPE 未指定だと GCC は最適化なし（-O0）になり、
# 経路計画のような毎周期の計算が桁違いに遅くなる。CI と docker は明示的に Release を
# 指定しているので、これはローカルビルドを同じ条件に揃えるためのもの。
# デバッグ時は -DCMAKE_BUILD_TYPE=Debug または RelWithDebInfo を明示する。
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
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

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(
  include
  ${robocup_ssl_msgs_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# Game Controller Component
ament_auto_add_library(game_controller_component SHARED
  src/game_controller_component.cpp
)
ament_target_dependencies(game_controller_component
  robocup_ssl_msgs
)
target_link_libraries(game_controller_component
  ${robocup_ssl_msgs_LIBRARIES}
  robocup_ssl_msgs::robocup_ssl_msgs_conversions
  robocup_ssl_msgs::robocup_ssl_msgs_proto
)

ament_auto_add_executable(game_controller_node
  src/game_controller_node.cpp
)
target_link_libraries(game_controller_node game_controller_component)

rclcpp_components_register_nodes(game_controller_component "robocup_ssl_comm::GameController")

# Tracker Component
ament_auto_add_library(tracker_component SHARED
  src/tracker_component.cpp
)
target_link_libraries(tracker_component ${robocup_ssl_msgs_LIBRARIES})

ament_auto_add_executable(tracker_node
  src/tracker_node.cpp
)
target_link_libraries(tracker_node tracker_component)

rclcpp_components_register_nodes(tracker_component "robocup_ssl_comm::Tracker")

# Vision Component
ament_auto_add_library(vision_component SHARED
  src/vision_component.cpp
)
target_link_libraries(vision_component ${robocup_ssl_msgs_LIBRARIES})

ament_auto_add_executable(vision_node
  src/vision_node.cpp
)
target_link_libraries(vision_node vision_component)

rclcpp_components_register_nodes(vision_component "robocup_ssl_comm::Vision")

ament_export_libraries(vision_component)
ament_export_libraries(game_controller_component)
ament_export_libraries(tracker_component)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE launch USE_SCOPED_HEADER_INSTALL_DIR)
