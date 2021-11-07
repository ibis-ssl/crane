cmake_minimum_required(VERSION 3.5)
project(crane_play_switcher)

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
find_package(consai2r2_msgs REQUIRED)
find_package(crane_msgs REQUIRED)

include_directories(include ${CMAKE_CURRENT_BINARY_DIR})
add_library(play_switcher SHARED src/play_switcher.cpp)

ament_target_dependencies(play_switcher
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
  "consai2r2_msgs"
  "crane_msgs"
)

rclcpp_components_register_nodes(play_switcher "crane::PlaySwitcher")

add_executable(play_switcher_node
  src/play_switcher_node.cpp
)
target_link_libraries(play_switcher_node play_switcher)
ament_target_dependencies(play_switcher_node
  "rclcpp"
  "std_msgs"
  "consai2r2_msgs"
  "crane_msgs"
)
install(TARGETS play_switcher_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS play_switcher_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
