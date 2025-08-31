cmake_minimum_required(VERSION 3.5)
project(crane_local_planner)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g -Wno-unused-parameter -Wno-unused-variable)
endif()

add_definitions("-DBOOST_ALLOW_DEPRECATED_HEADERS")

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(TBB REQUIRED)
find_package(Eigen3 REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME}_component SHARED
  src/graph_planner.cpp
  src/rvo2_planner.cpp
  src/modern_orca_planner.cpp
  src/crane_local_planner.cpp
)

target_link_libraries(${PROJECT_NAME}_component glog TBB::tbb Eigen3::Eigen)

# target_precompile_headers(${PROJECT_NAME}_component
#   PUBLIC
#     "<crane_geometry/boost_geometry.hpp>"
#     "<crane_physics/pid_controller.hpp>"
#     "<crane_msgs/msg/robot_commands.hpp>"
#     "<crane_msg_wrappers/world_model_wrapper.hpp>"
#     "<crane_msg_wrappers/crane_visualizer_wrapper.hpp>"
#     "<rclcpp/rclcpp.hpp>"
# )

rclcpp_components_register_nodes(${PROJECT_NAME}_component "crane::LocalPlannerComponent")

ament_auto_add_executable(${PROJECT_NAME}_node
  src/${PROJECT_NAME}_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node TBB::tbb)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_modern_orca_planner
    test/test_modern_orca_planner.cpp
  )

  target_link_libraries(test_modern_orca_planner
    ${PROJECT_NAME}_component
    TBB::tbb
    Eigen3::Eigen
  )

  ament_target_dependencies(test_modern_orca_planner
    rclcpp
    crane_msgs
    crane_msg_wrappers
  )
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
