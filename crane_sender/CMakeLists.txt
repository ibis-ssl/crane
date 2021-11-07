cmake_minimum_required(VERSION 3.5)
project(crane_sender)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(crane_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(crane_protobuf REQUIRED)
find_package(Protobuf REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

include_directories(
  ${Boost_INCLUDE_DIRS}
  include
  ${CMAKE_CURRENT_BINARY_DIR}
  ${crane_protobuf_INCLUDE_DIRS})

add_executable(real_sender_node
  src/real_sender_node.cpp
)

add_executable(sim_sender_node
  src/sim_sender_node.cpp
)

target_link_libraries(sim_sender_node ${Boost_LIBRARIES} ${crane_protobuf_LIBRARIES})

ament_target_dependencies(
  real_sender_node
  "rclcpp"
  "std_msgs"
  "crane_msgs"
  "sensor_msgs"
)
ament_target_dependencies(
  sim_sender_node
  "rclcpp"
  "std_msgs"
  "crane_msgs"
  "sensor_msgs"
)

install(TARGETS real_sender_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS sim_sender_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
