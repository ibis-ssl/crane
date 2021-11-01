cmake_minimum_required(VERSION 3.5)
project(crane_world_observer)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(PkgConfig)
pkg_check_modules(BFL REQUIRED orocos-bfl)
include_directories(${BFL_INCLUDE_DIRS})
link_directories(${BFL_LIBRARY_DIRS})
link_libraries(${BFL_LIBRARIES})

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(consai2r2_msgs REQUIRED)
find_package(crane_msgs REQUIRED)
find_package(crane_geometry REQUIRED)
find_package(crane_msg_wrappers REQUIRED)

include_directories(include ${CMAKE_CURRENT_BINARY_DIR})

add_executable(world_observer
  src/world_observer_node.cpp
  src/estimator/estimator.cpp
)


ament_target_dependencies(
  world_observer
  "rclcpp"
  "consai2r2_msgs"
  "crane_msgs"
  "crane_geometry"
  "crane_msg_wrappers"
)

install(TARGETS world_observer
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY "include/"
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_package()
