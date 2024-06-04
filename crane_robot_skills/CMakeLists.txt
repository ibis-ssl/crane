cmake_minimum_required(VERSION 3.5)
project(crane_robot_skills)

# Default to C++20
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif ()

add_definitions("-DBOOST_ALLOW_DEPRECATED_HEADERS")

find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp_components REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME}_component SHARED
  src/robot_skills.cpp
  src/robot_command_as_skill.cpp
  src/get_ball_contact.cpp
  src/go_over_ball.cpp
  src/goalie.cpp
  src/move_with_ball.cpp
  src/receiver.cpp
  src/simple_attacker.cpp
  src/single_ball_placement.cpp
)

#rclcpp_components_register_nodes(${PROJECT_NAME}_component "crane::WorldModelPublisherComponent")

#ament_auto_add_executable(${PROJECT_NAME}_node src/robot_skills_node.cpp)
#target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_component)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
