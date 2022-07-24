cmake_minimum_required(VERSION 3.5)
project(crane_waiter_planner)

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

ament_auto_add_library(${PROJECT_NAME}_component SHARED
        src/crane_waiter_planner.cpp include/crane_waiter_planner/waiter_planner.hpp)

rclcpp_components_register_nodes(${PROJECT_NAME}_component "crane::WaiterPlanner")

ament_auto_add_executable(${PROJECT_NAME}_node
        src/${PROJECT_NAME}_node.cpp
        )

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_auto_package()
