cmake_minimum_required(VERSION 3.8)
project(crane_visualization_interfaces)

add_compile_options(-g)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(proto2ros REQUIRED)

find_package(Protobuf REQUIRED)

add_subdirectory(proto)

### proto -> ROS msg
proto2ros_generate(
    ${PROJECT_NAME}_messages_gen
    PROTOS proto/ObjectsArray.proto
    IMPORT_DIRS proto
    INTERFACES_OUT_VAR ros_messages
    PYTHON_OUT_VAR py_sources
    CPP_OUT_VAR cpp_sources
    INCLUDE_OUT_VAR cpp_include_dir
    APPEND_PYTHONPATH "${CMAKE_CURRENT_BINARY_DIR}/proto"
    DEPENDS ${PROJECT_NAME}_proto_gen
)

## ROS msg -> C++ / Python
rosidl_generate_interfaces(
    ${PROJECT_NAME} ${ros_messages}
    DEPENDENCIES builtin_interfaces proto2ros
)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_messages_gen)

##　変換ライブラリ
add_library(${PROJECT_NAME}_conversions SHARED ${cpp_sources} src/manual_conversions.cpp)
target_include_directories(${PROJECT_NAME}_conversions PUBLIC
    "$<BUILD_INTERFACE:${cpp_include_dir}>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/proto>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
rosidl_get_typesupport_target(${PROJECT_NAME}_cpp_msgs ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_conversions
    ${${PROJECT_NAME}_cpp_msgs} ${PROJECT_NAME}_proto protobuf::libprotobuf)
ament_target_dependencies(${PROJECT_NAME}_conversions
    builtin_interfaces proto2ros rclcpp)

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

#install(
#    FILES ${CMAKE_CURRENT_BINARY_DIR}/proto/ObjectsArray.pb.h
#    DESTINATION include/${PROJECT_NAME}/
#)

# ライブラリインストール
install(
    TARGETS
    ${PROJECT_NAME}_proto
    ${PROJECT_NAME}_conversions
    EXPORT ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
ament_export_dependencies(builtin_interfaces proto2ros rclcpp)
ament_export_targets(${PROJECT_NAME})

ament_package()
