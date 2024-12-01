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
  DIRECTORY src include
)

target_precompile_headers(${PROJECT_NAME}_component
  PUBLIC
    "<crane_basics/boost_geometry.hpp>"
    "<crane_basics/eigen_adapter.hpp>"
    "<crane_robot_skills/skill_base.hpp>"
    "<Eigen/Geometry>"
    "<crane_robot_skills/receive.hpp>"
    "<crane_robot_skills/kick.hpp>"
)

#rclcpp_components_register_nodes(${PROJECT_NAME}_component "crane::WorldModelPublisherComponent")

#ament_auto_add_executable(${PROJECT_NAME}_node src/robot_skills_node.cpp)
#target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_component)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
