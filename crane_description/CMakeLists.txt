cmake_minimum_required(VERSION 3.12)
project(crane_description)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

set(_PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}")

if(WIN32 AND CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE_DEBUG}")
endif()

ament_python_install_package(${PROJECT_NAME})

include_directories(include ${CMAKE_CURRENT_BINARY_DIR})
add_executable(${PROJECT_NAME}_node
  src/${PROJECT_NAME}_node.cpp
)
ament_target_dependencies(
  ${PROJECT_NAME}_node
  "rclcpp"
)


install(TARGETS ${PROJECT_NAME}_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  include
  DESTINATION .
)

ament_export_include_directories(include)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(python_cmake_module)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

set(PYTHON_EXECUTABLE "${_PYTHON_EXECUTABLE}")

ament_package()
