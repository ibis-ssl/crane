cmake_minimum_required(VERSION 3.12)
project(crane_local_planner)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter -Wno-unused-variable)
  if(NOT CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_options(-g)
  endif()
endif()

add_definitions("-DBOOST_ALLOW_DEPRECATED_HEADERS")

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(TBB REQUIRED)
find_package(Eigen3 REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME}_component SHARED
  src/rvo2_planner.cpp
  src/crane_local_planner.cpp
  src/ateb_planner.cpp
  src/ateb_visibility_graph.cpp
  src/ateb_spatial_optimizer.cpp
  src/ateb_time_parameterizer.cpp
  src/ateb_cbf_filter.cpp
)

option(CRANE_UNITY_BUILD "Enable unity build" ON)
if(CRANE_UNITY_BUILD)
  set_target_properties(${PROJECT_NAME}_component PROPERTIES UNITY_BUILD ON)
  set_target_properties(${PROJECT_NAME}_component PROPERTIES UNITY_BUILD_BATCH_SIZE 2)
endif()

target_link_libraries(${PROJECT_NAME}_component glog TBB::tbb Eigen3::Eigen)

rclcpp_components_register_nodes(${PROJECT_NAME}_component "crane::LocalPlannerComponent")

ament_auto_add_executable(${PROJECT_NAME}_node
  src/${PROJECT_NAME}_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node TBB::tbb)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_auto_add_gtest(test_obstacle_distance test/test_obstacle_distance.cpp)
  target_link_libraries(test_obstacle_distance ${PROJECT_NAME}_component Eigen3::Eigen)

  ament_auto_add_gtest(test_visibility_graph test/test_visibility_graph.cpp)
  target_link_libraries(test_visibility_graph ${PROJECT_NAME}_component Eigen3::Eigen)

  ament_auto_add_gtest(test_spatial_optimizer test/test_spatial_optimizer.cpp)
  target_link_libraries(test_spatial_optimizer ${PROJECT_NAME}_component Eigen3::Eigen)

  ament_auto_add_gtest(test_cbf_filter test/test_cbf_filter.cpp)
  target_link_libraries(test_cbf_filter ${PROJECT_NAME}_component Eigen3::Eigen)

  ament_auto_add_gtest(test_ateb_integration test/test_ateb_integration.cpp)
  target_link_libraries(test_ateb_integration ${PROJECT_NAME}_component Eigen3::Eigen)
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
