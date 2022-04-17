cmake_minimum_required(VERSION 3.5)
project(crane_game_analyzer)

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
find_package(std_msgs REQUIRED)
find_package(crane_msgs REQUIRED)
find_package(crane_msg_wrappers REQUIRED)

include_directories(include ${CMAKE_CURRENT_BINARY_DIR})
add_library(game_analyzer SHARED src/game_analyzer.cpp)

ament_target_dependencies(game_analyzer
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
  "crane_msgs"
  "crane_msg_wrappers"
)

rclcpp_components_register_nodes(game_analyzer "crane::GameAnalyzer")

add_executable(game_analyzer_node
  src/game_analyzer_node.cpp
)
target_link_libraries(game_analyzer_node game_analyzer)
ament_target_dependencies(game_analyzer_node
  "rclcpp"
  "std_msgs"
  "crane_msgs"
  "crane_msg_wrappers"
)
install(TARGETS game_analyzer_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS game_analyzer_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
