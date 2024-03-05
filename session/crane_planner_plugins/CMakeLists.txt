cmake_minimum_required(VERSION 3.5)
project(crane_planner_plugins)

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif ()

add_definitions("-DBOOST_ALLOW_DEPRECATED_HEADERS")

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp_components REQUIRED)
ament_auto_find_build_dependencies()

# for matplotlib
find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
find_package(PythonLibs 3.0 REQUIRED)
include_directories(${PYTHON3_INCLUDE_DIRS} ${NumPy_INCLUDE_DIRS})

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src include
)

ament_auto_add_executable(receive_test_node src/receive_test.cpp)
ament_target_dependencies(receive_test_node "Python3")
target_link_libraries(receive_test_node ${PYTHON_LIBRARIES} ${PROJECT_NAME})


if (BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif ()

ament_auto_package()
