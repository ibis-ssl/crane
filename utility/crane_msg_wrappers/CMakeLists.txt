cmake_minimum_required(VERSION 3.12)
project(crane_msg_wrappers)


# Default to Release build if not specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()
# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  if(NOT CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_options(-g)
  endif()
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
    "<range/v3/action/sort.hpp>"
    "<range/v3/action/remove_if.hpp>"
    "<range/v3/algorithm/any_of.hpp>"
    "<range/v3/algorithm/contains.hpp>"
    "<range/v3/algorithm/copy.hpp>"
    "<range/v3/algorithm/count.hpp>"
    "<range/v3/algorithm/find_if.hpp>"
    "<range/v3/algorithm/for_each.hpp>"
    "<range/v3/algorithm/max.hpp>"
    "<range/v3/algorithm/max_element.hpp>"
    "<range/v3/algorithm/min.hpp>"
    "<range/v3/algorithm/min_element.hpp>"
    "<range/v3/functional/comparisons.hpp>"
    "<range/v3/iterator/insert_iterators.hpp>"
    "<range/v3/range/conversion.hpp>"
    "<range/v3/view/enumerate.hpp>"
    "<range/v3/view/filter.hpp>"
    "<range/v3/view/iota.hpp>"
    "<range/v3/view/join.hpp>"
    "<range/v3/view/take_while.hpp>"
    "<range/v3/view/transform.hpp>"
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
