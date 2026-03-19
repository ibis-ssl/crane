cmake_minimum_required(VERSION 3.12)
project(crane_bag)

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

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Bag Analysis CLI Tool
ament_auto_add_executable(crane_bag
  src/bag/crane_bag_main.cpp
  src/bag/bag_reader.cpp
  src/bag/bag_survey.cpp
  src/bag/bag_events.cpp
  src/bag/bag_tracking.cpp
  src/bag/bag_control.cpp
  src/bag/bag_referee.cpp
)
target_include_directories(crane_bag PRIVATE src/bag)

ament_auto_package()
