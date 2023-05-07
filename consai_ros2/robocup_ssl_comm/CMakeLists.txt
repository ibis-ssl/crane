cmake_minimum_required(VERSION 3.5)
project(robocup_ssl_comm)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

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

include_directories(
  include
  ${robocup_ssl_msgs_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# Vision Component
ament_auto_add_library(vision_component SHARED
  src/vision_component.cpp
)

ament_auto_add_executable(vision_node
  src/vision_node.cpp
)
target_link_libraries(vision_node vision_component)

rclcpp_components_register_nodes(vision_component "robocup_ssl_comm::Vision")
target_link_libraries(vision_component
  ${robocup_ssl_msgs_LIBRARIES} ${Boost_LIBRARIES}
)


# Game Controller Component
ament_auto_add_library(game_controller_component SHARED
  src/game_controller_component.cpp
)

ament_auto_add_executable(game_controller_node
  src/game_controller_node.cpp
)
target_link_libraries(game_controller_node game_controller_component)

rclcpp_components_register_nodes(game_controller_component "robocup_ssl_comm::GameController")

# GrSim Component
ament_auto_add_library(grsim_component SHARED
  src/grsim_component.cpp
)

ament_auto_add_executable(grsim_node
  src/grsim_node.cpp
)
target_link_libraries(grsim_node grsim_component)

rclcpp_components_register_nodes(grsim_component "robocup_ssl_comm::GrSim")

ament_export_libraries(vision_component)
ament_export_libraries(game_controller_component)
ament_export_libraries(grsim_component)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE launch)
