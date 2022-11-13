cmake_minimum_required(VERSION 3.5)
project(crane_msg_wrappers)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# for FastRTPS error
set(CMAKE_PREFIX_PATH /opt/ros/foxy/)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(PkgConfig)
pkg_check_modules(BFL REQUIRED orocos-bfl)
include_directories(${BFL_INCLUDE_DIRS})
link_directories(${BFL_LIBRARY_DIRS})
link_libraries(${BFL_LIBRARIES})

# find dependencies
find_package(ament_cmake_auto REQUIRED)

set(requirement_dependencies
      "rclcpp"
      "tf2"
      "std_msgs"
      "geometry_msgs"
      "nav_msgs"
      "crane_msgs"
      "robocup_ssl_msgs"
      "crane_geometry"
      "Eigen3"
)

ament_auto_find_build_dependencies(REQUIRED ${requirement_dependencies})

include_directories(include)

ament_auto_add_library(${PROJECT_NAME}
        src/geometry_wrapper.cpp)


ament_target_dependencies(${PROJECT_NAME} ${requirement_dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
