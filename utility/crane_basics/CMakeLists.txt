cmake_minimum_required(VERSION 3.5)
project(crane_basics)

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
find_package(Eigen3 REQUIRED)
ament_auto_find_build_dependencies()

ament_export_include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(test_geometry test/test.cpp)
  target_include_directories(test_geometry PUBLIC include)
  target_include_directories(test_geometry PUBLIC ${EIGEN3_INCLUDE_DIRS})
  target_include_directories(test_geometry PUBLIC ${closest_point_vendor_INCLUDE_DIRS})
endif()



ament_auto_package()
