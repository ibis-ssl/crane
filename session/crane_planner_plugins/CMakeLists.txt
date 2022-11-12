cmake_minimum_required(VERSION 3.5)
project(crane_planner_plugins)

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp_components REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(ball_placement SHARED src/ball_placement/ball_placement.cpp)
rclcpp_components_register_nodes(ball_placement "crane::BallPlacementPlanner")
ament_auto_add_executable(ball_placement_node src/ball_placement/ball_placement_node.cpp)
target_link_libraries(ball_placement_node ball_placement)

pluginlib_export_plugin_description_file(crane_planner_base plugins.xml)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_auto_package()
