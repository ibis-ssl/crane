cmake_minimum_required(VERSION 3.5)
project(crane_world_model_publisher)

# Default to C++20
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif ()

add_definitions("-DBOOST_ALLOW_DEPRECATED_HEADERS")

find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(Eigen3 REQUIRED) # Explicitly find Eigen3
ament_auto_find_build_dependencies(REQUIRED Eigen3) # Add Eigen3 here

ament_auto_add_library(${PROJECT_NAME}_component SHARED
  src/world_model_publisher.cpp
  src/visualization_data_handler.cpp
  src/world_model_data_provider.cpp
)

target_link_libraries(${PROJECT_NAME}_component
        ${robocup_ssl_msgs_LIBRARIES} ${Boost_LIBRARIES} Eigen3::Eigen # Add Eigen3::Eigen here
)

rclcpp_components_register_nodes(${PROJECT_NAME}_component "crane::WorldModelPublisherComponent")

ament_auto_add_executable(${PROJECT_NAME}_node src/world_model_publisher_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_component glog)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
