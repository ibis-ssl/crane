cmake_minimum_required(VERSION 3.12)
project(crane_msg_wrappers)

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
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED DIRECTORY src)

# プリコンパイル済みヘッダでビルド時間を短縮
target_precompile_headers(${PROJECT_NAME}
  PUBLIC
    "<crane_geometry/boost_geometry.hpp>"
    "<crane_geometry/geometry_operations.hpp>"
    "<crane_msgs/msg/world_model.hpp>"
    "<crane_msgs/msg/robot_command.hpp>"
    "<rclcpp/rclcpp.hpp>"
    "<range/v3/all.hpp>"
    "<Eigen/Core>"
    "<Eigen/Geometry>"
    "<memory>"
    "<vector>"
    "<string>"
    "<map>"
    "<functional>"
)

ament_target_dependencies(${PROJECT_NAME} ${requirement_dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
