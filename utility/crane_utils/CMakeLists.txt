cmake_minimum_required(VERSION 3.12)
project(crane_utils)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_export_include_directories(
  include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Utility tests
  ament_auto_add_gtest(test_time test/test_time.cpp)
  target_include_directories(test_time PUBLIC include)
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
