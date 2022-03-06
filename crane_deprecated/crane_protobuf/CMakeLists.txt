cmake_minimum_required(VERSION 3.5)
project(crane_protobuf)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Protobuf REQUIRED)
find_package(python_cmake_module REQUIRED)

set(_PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}")

if(WIN32 AND CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE_DEBUG}")
endif()

ament_python_install_package(${PROJECT_NAME})

# Protobuf
include_directories(
  ${PROTOBUF_INCLUDE_DIRS}
)

set(protobuf_files
  proto/messages_robocup_ssl_wrapper.proto
  proto/messages_robocup_ssl_geometry.proto
  proto/messages_robocup_ssl_detection.proto
  proto/messages_robocup_ssl_robot_status.proto
  proto/messages_robocup_ssl_refbox_log.proto
  proto/referee.proto
  proto/game_event.proto
  proto/grSim_Replacement.proto
  proto/grSim_Commands.proto
  proto/grSim_Packet.proto
)

protobuf_generate_cpp(PROTO_CPP PROTO_H
  ${protobuf_files}
)

include_directories(include ${CMAKE_CURRENT_BINARY_DIR})

add_library(crane_protobuf
  SHARED ${PROTO_CPP} ${PROTO_H}
)

target_link_libraries(crane_protobuf ${PROTOBUF_LIBRARIES})

protobuf_generate_python(PROTO_PY
  ${protobuf_files}
)

add_custom_target(crane_protobuf_py ALL DEPENDS ${PROTO_PY})

install(
  FILES ${PROTO_H}
  DESTINATION include/${PROJECT_NAME})

install(
  TARGETS crane_protobuf
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

install(FILES ${PROTO_PY}
  DESTINATION "${PYTHON_INSTALL_DIR}")

ament_export_include_directories(include)
ament_export_libraries(crane_protobuf)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(Protobuf)
ament_export_dependencies(python_cmake_module)


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

ament_package()
