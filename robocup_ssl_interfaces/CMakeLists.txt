cmake_minimum_required(VERSION 3.5)
project(robocup_ssl_interfaces)

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

find_package(Protobuf REQUIRED)

file(GLOB proto_files "proto/*.proto")
message(WARNING ${proto_files})

#add_subdirectory(proto)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

protobuf_generate(
        LANGUAGE python
        OUT_VAR proto_py_sources
        PROTOS ${proto_files}
)

message(WARNING "protobuf_generate python!")

protobuf_generate(
        LANGUAGE cpp
        OUT_VAR proto_cpp_sources
        PROTOS ${proto_files}
)

message(WARNING "protobuf_generate cpp!")

add_library(${PROJECT_NAME}_proto SHARED ${proto_cpp_sources})
#target_include_directories(${PROJECT_NAME}_proto PUBLIC "${CMAKE_CURRENT_BINARY_DIR}")
target_compile_options(${PROJECT_NAME}_proto PUBLIC -Wno-deprecated-declarations)
target_link_libraries(${PROJECT_NAME}_proto protobuf::libprotobuf)

add_custom_target(
        ${PROJECT_NAME}_proto_gen ALL DEPENDS
        ${proto_py_sources}
        ${proto_cpp_sources}
)

### proto -> ROS msg
proto2ros_generate(
        ${PROJECT_NAME}_messages_gen
        PROTOS ${proto_files}
#        IMPORT_DIRS proto
        INTERFACES_OUT_VAR ros_messages
        PYTHON_OUT_VAR py_sources
        CPP_OUT_VAR cpp_sources
        INCLUDE_OUT_VAR cpp_include_dir
        APPEND_PYTHONPATH "${CMAKE_CURRENT_BINARY_DIR}/proto"
        DEPENDS ${PROJECT_NAME}_proto_gen
)

message(WARNING "proto2ros_generate!")

## ROS msg -> C++ / Python
rosidl_generate_interfaces(
        ${PROJECT_NAME} ${ros_messages}
        DEPENDENCIES builtin_interfaces proto2ros
)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_messages_gen)

message(WARNING "rosidl_generate_interfaces!")

rosidl_generated_python_package_add(
        ${PROJECT_NAME}_additional_modules
        MODULES ${proto_py_sources} ${py_sources}
        PACKAGES ${PROJECT_NAME}
        DESTINATION ${PROJECT_NAME}
)

# C++ファイルインストール
set(cpp_headers ${cpp_sources} ${proto_cpp_sources})
list(FILTER cpp_headers INCLUDE REGEX ".*\.(hpp|h)$")
install(
        FILES ${cpp_headers}
        DESTINATION include/${PROJECT_NAME}/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
