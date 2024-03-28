cmake_minimum_required(VERSION 3.5)
project(crane_local_planner)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif()

add_definitions("-DBOOST_ALLOW_DEPRECATED_HEADERS")

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp_components REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME}_component SHARED DIRECTORY src)

rclcpp_components_register_nodes(${PROJECT_NAME}_component "crane::LocalPlannerComponent")

ament_auto_add_executable(${PROJECT_NAME}_node
  src/${PROJECT_NAME}_node.cpp
)
add_backward(${PROJECT_NAME}_node)

# for matplotlib
find_package(Python COMPONENTS Interpreter Development NumPy REQUIRED)
include_directories(${Python_INCLUDE_DIRS} ${NumPy_INCLUDE_DIRS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(test_node test/test.cpp)
  ament_target_dependencies(test_node "Python3")
  target_link_libraries(test_node ${PYTHON_LIBRARIES} ${PROJECT_NAME}_component)
endif()

ament_auto_package()
