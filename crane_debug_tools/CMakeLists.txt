cmake_minimum_required(VERSION 3.12)
project(crane_debug_tools)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Simple Web Server (temporary solution)
ament_auto_add_executable(crane_web_server
  src/simple_web_server.cpp
)

# WebSocket Debug Server
ament_auto_add_executable(crane_websocket_server
  src/websocket_server.cpp
)

# Find dependencies for WebSocket server
find_package(OpenSSL REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# Link libraries for WebSocket server
target_link_libraries(crane_websocket_server ${Boost_LIBRARIES})
if(TARGET OpenSSL::SSL)
  target_link_libraries(crane_websocket_server OpenSSL::SSL OpenSSL::Crypto)
else()
  target_link_libraries(crane_websocket_server ${OPENSSL_LIBRARIES})
  target_include_directories(crane_websocket_server PRIVATE ${OPENSSL_INCLUDE_DIR})
endif()

# Install
ament_auto_package(INSTALL_TO_SHARE web launch)
