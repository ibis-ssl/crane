cmake_minimum_required(VERSION 3.12)
project(robocup_ssl_msgs)


# Default to Release build if not specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()
# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g -Wno-deprecated-declarations)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(proto2ros REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Protobuf REQUIRED)

# List all proto files
file(GLOB PROTO_FILES "proto/*.proto")

# Build protobuf C++ and Python libraries
add_subdirectory(proto)

# Generate ROS messages and conversion code from proto files using proto2ros
# Convert absolute paths to relative paths for PROTOS parameter
set(PROTO_FILES_RELATIVE)
foreach(proto_file ${PROTO_FILES})
  file(RELATIVE_PATH proto_relative "${CMAKE_CURRENT_SOURCE_DIR}" "${proto_file}")
  list(APPEND PROTO_FILES_RELATIVE ${proto_relative})
endforeach()

proto2ros_generate(
  ${PROJECT_NAME}_messages_gen
  PROTOS ${PROTO_FILES_RELATIVE}
  IMPORT_DIRS proto
  INTERFACES_OUT_VAR ros_messages
  PYTHON_OUT_VAR py_sources
  CPP_OUT_VAR cpp_sources
  INCLUDE_OUT_VAR cpp_include_dir
  APPEND_PYTHONPATH "${CMAKE_CURRENT_BINARY_DIR}/proto"
  DEPENDS ${PROJECT_NAME}_proto_gen
)

# Generate ROS interface definitions
rosidl_generate_interfaces(
  ${PROJECT_NAME} ${ros_messages}
  DEPENDENCIES builtin_interfaces std_msgs proto2ros
)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_messages_gen)

# Build conversion library
add_library(${PROJECT_NAME}_conversions SHARED ${cpp_sources})
set_target_properties(${PROJECT_NAME}_conversions PROPERTIES
  CXX_STANDARD 20
  CXX_STANDARD_REQUIRED ON
)

target_include_directories(${PROJECT_NAME}_conversions PUBLIC
  "$<BUILD_INTERFACE:${cpp_include_dir}>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

# Get ROS message typesupport target
rosidl_get_typesupport_target(
  ${PROJECT_NAME}_cpp_msgs
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

target_link_libraries(${PROJECT_NAME}_conversions
  ${${PROJECT_NAME}_cpp_msgs}
  ${PROJECT_NAME}_proto
  protobuf::libprotobuf
)

ament_target_dependencies(${PROJECT_NAME}_conversions
  builtin_interfaces
  std_msgs
  proto2ros
)

# Install conversion library
install(
  TARGETS ${PROJECT_NAME}_conversions
  EXPORT ${PROJECT_NAME}_conversions
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install generated conversion headers
install(
  DIRECTORY ${cpp_include_dir}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
)

# Export include directories
ament_export_include_directories(include)

# Export dependencies
ament_export_targets(${PROJECT_NAME}_proto HAS_LIBRARY_TARGET)
ament_export_targets(${PROJECT_NAME}_conversions HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rosidl_default_runtime
  builtin_interfaces
  std_msgs
  proto2ros
  Protobuf
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
