cmake_minimum_required(VERSION 3.12)
project(crane_web_debugger)

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

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()

# WebSocket Debug Server
ament_auto_add_executable(crane_websocket_server
  src/websocket_server.cpp
)

find_package(OpenSSL REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Protobuf REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(${robocup_ssl_msgs_INCLUDE_DIRS})

target_link_libraries(crane_websocket_server ${Boost_LIBRARIES} ${robocup_ssl_msgs_LIBRARIES} ${YAML_CPP_LIBRARIES})
target_link_libraries(crane_websocket_server OpenSSL::SSL OpenSSL::Crypto)

ament_auto_package(INSTALL_TO_SHARE web)
