# Python関連の挙動が3.12を境に変わっているらしい
# 参考: https://qiita.com/mitsu48/items/fc4e1779be0cfe14091d
cmake_minimum_required(VERSION 3.12)
project(crane_sessions)

# 既定を Release にする。CMAKE_BUILD_TYPE 未指定だと GCC は最適化なし（-O0）になり、
# 経路計画のような毎周期の計算が桁違いに遅くなる。CI と docker は明示的に Release を
# 指定しているので、これはローカルビルドを同じ条件に揃えるためのもの。
# デバッグ時は -DCMAKE_BUILD_TYPE=Debug または RelWithDebInfo を明示する。
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

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
find_package(yaml-cpp REQUIRED)
find_package(magic_enum REQUIRED)
ament_auto_find_build_dependencies()

find_package(Python COMPONENTS Interpreter Development NumPy REQUIRED)
include_directories(${Python_INCLUDE_DIRS} ${NumPy_INCLUDE_DIRS})

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src include
)

target_link_libraries(${PROJECT_NAME} ${YAML_CPP_LIBRARIES} magic_enum::magic_enum)

# Enable Unity Build for faster compilation
option(CRANE_UNITY_BUILD "Enable unity build" ON)
if(CRANE_UNITY_BUILD)
  set_target_properties(${PROJECT_NAME} PROPERTIES UNITY_BUILD ON)
  set_target_properties(${PROJECT_NAME} PROPERTIES UNITY_BUILD_BATCH_SIZE 6)
endif()

if (BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif ()

ament_auto_package(INSTALL_TO_SHARE config USE_SCOPED_HEADER_INSTALL_DIR)
