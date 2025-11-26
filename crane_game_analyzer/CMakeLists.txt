cmake_minimum_required(VERSION 3.12)
project(crane_game_analyzer)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif()

add_definitions("-DBOOST_ALLOW_DEPRECATED_HEADERS")

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp_components REQUIRED)
ament_auto_find_build_dependencies()

message("closest_point_vendor_DIR")
message(${closest_point_vendor_DIR})
ament_auto_add_library(${PROJECT_NAME}_component SHARED src/crane_game_analyzer.cpp src/evaluations.cpp)

target_precompile_headers(${PROJECT_NAME}_component
  PUBLIC
    "<crane_geometry/boost_geometry.hpp>"
    "<crane_geometry/geometry_operations.hpp>"
    "<crane_msg_wrappers/world_model_wrapper.hpp>"
    "<crane_msgs/msg/world_model.hpp>"
    "<rclcpp/rclcpp.hpp>"
    "<algorithm>"
    "<vector>"
    "<cmath>"
)

ament_auto_add_executable(${PROJECT_NAME}_node
  src/${PROJECT_NAME}_node.cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
