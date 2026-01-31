cmake_minimum_required(VERSION 3.12)
project(crane_robot_skills)


# Default to Release build if not specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()
# Default to C++20
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  if(NOT CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_options(-g)
  endif()
endif ()

add_definitions("-DBOOST_ALLOW_DEPRECATED_HEADERS")

find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp_components REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME}_component SHARED
  DIRECTORY src include
)

# Enable Unity Build for faster compilation
option(CRANE_UNITY_BUILD "Enable unity build" ON)
if(CRANE_UNITY_BUILD)
  set_target_properties(${PROJECT_NAME}_component PROPERTIES UNITY_BUILD ON)
  set_target_properties(${PROJECT_NAME}_component PROPERTIES UNITY_BUILD_BATCH_SIZE 8)
endif()

target_precompile_headers(${PROJECT_NAME}_component
  PRIVATE
    "<Eigen/Core>"
    "<Eigen/Geometry>"
    "<array>"
    "<optional>"
    "<vector>"
    "<string>"
    "<format>"
    "<crane_geometry/boost_geometry.hpp>"
    "<crane_geometry/geometry_operations.hpp>"
    "<crane_msg_wrappers/world_model_wrapper.hpp>"
    "<crane_msg_wrappers/robot_command_wrapper.hpp>"
    "<crane_msg_wrappers/crane_visualizer_wrapper.hpp>"
    "<crane_msgs/msg/world_model.hpp>"
    "<crane_physics/ball_info.hpp>"
    "<crane_physics/robot_info.hpp>"
    "<rclcpp/rclcpp.hpp>"
)

target_precompile_headers(${PROJECT_NAME}_component
  PUBLIC
    "<crane_geometry/boost_geometry.hpp>"
    "<crane_robot_skills/skill_base.hpp>"
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

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
