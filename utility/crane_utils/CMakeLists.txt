cmake_minimum_required(VERSION 3.12)
project(crane_utils)

# 既定を Release にする。CMAKE_BUILD_TYPE 未指定だと GCC は最適化なし（-O0）になり、
# 経路計画のような毎周期の計算が桁違いに遅くなる。CI と docker は明示的に Release を
# 指定しているので、これはローカルビルドを同じ条件に揃えるためのもの。
# デバッグ時は -DCMAKE_BUILD_TYPE=Debug または RelWithDebInfo を明示する。
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
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

  ament_auto_add_gtest(test_parameter test/test_parameter.cpp)
  target_include_directories(test_parameter PUBLIC include)

  ament_auto_add_gtest(test_package test/test_package.cpp)
  target_include_directories(test_package PUBLIC include)
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
