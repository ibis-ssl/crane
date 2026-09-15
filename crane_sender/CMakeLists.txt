cmake_minimum_required(VERSION 3.12)
project(crane_sender)

# 既定を Release にする。CMAKE_BUILD_TYPE 未指定だと GCC は最適化なし（-O0）になり、
# 経路計画のような毎周期の計算が桁違いに遅くなる。CI と docker は明示的に Release を
# 指定しているので、これはローカルビルドを同じ条件に揃えるためのもの。
# デバッグ時は -DCMAKE_BUILD_TYPE=Debug または RelWithDebInfo を明示する。
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

# Default to C++20
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)

  # Debug symbols only for Debug builds
  if(CMAKE_BUILD_TYPE STREQUAL "Debug" OR CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
    add_compile_options(-g)
  endif()
endif ()

add_definitions("-DBOOST_ALLOW_DEPRECATED_HEADERS")

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(
        include
        ${robocup_ssl_msgs_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
)

ament_auto_add_executable(ibis_sender_node src/ibis_sender_node.cpp)

add_library(crane_sender_base STATIC src/sender_base.cpp src/sim_position_controller.cpp)
set_target_properties(crane_sender_base PROPERTIES POSITION_INDEPENDENT_CODE ON)
ament_target_dependencies(crane_sender_base
  rclcpp
  crane_msgs
  crane_msg_wrappers
  crane_comm
  crane_utils
)
target_include_directories(crane_sender_base PUBLIC include)

target_link_libraries(ibis_sender_node crane_sender_base ${robocup_ssl_msgs_LIBRARIES})

if (BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_auto_add_gtest(test_robot_packet test/test_robot_packet.cpp)
  ament_auto_add_gtest(test_sim_position_controller test/test_sim_position_controller.cpp)
  target_link_libraries(test_sim_position_controller crane_sender_base)
endif ()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
