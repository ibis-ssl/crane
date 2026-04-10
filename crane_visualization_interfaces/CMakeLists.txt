cmake_minimum_required(VERSION 3.12)
project(crane_visualization_interfaces)

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

add_definitions("-DBOOST_ALLOW_DEPRECATED_HEADERS")

# === 依存パッケージ ===
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(crane_geometry REQUIRED)
find_package(Eigen3 REQUIRED)

# === メッセージ生成 ===
file(GLOB msg_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} CONFIGURE_DEPENDS "msg/*.msg")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# === ビジュアライザビルダーライブラリ ===
add_library(${PROJECT_NAME}_builder SHARED
  src/crane_visualizer_wrapper.cpp
)

target_include_directories(${PROJECT_NAME}_builder PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_link_libraries(${PROJECT_NAME}_builder
  ${cpp_typesupport_target}
  Eigen3::Eigen
)

ament_target_dependencies(${PROJECT_NAME}_builder
  rclcpp
  crane_geometry
)

# === アグリゲータコンポーネント ===
add_library(${PROJECT_NAME}_aggregator_component SHARED
  src/visualization_aggregator.cpp
)

target_include_directories(${PROJECT_NAME}_aggregator_component PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_link_libraries(${PROJECT_NAME}_aggregator_component
  ${cpp_typesupport_target}
)

ament_target_dependencies(${PROJECT_NAME}_aggregator_component
  rclcpp
  rclcpp_components
)

rclcpp_components_register_node(${PROJECT_NAME}_aggregator_component
  PLUGIN "crane::VisualizationAggregator"
  EXECUTABLE visualization_aggregator_node
)

# === インストール ===
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}_builder ${PROJECT_NAME}_aggregator_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# === エクスポート ===
ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rosidl_default_runtime
  rclcpp
  crane_geometry
  Eigen3
  builtin_interfaces
  std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
