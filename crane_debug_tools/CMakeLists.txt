cmake_minimum_required(VERSION 3.5)
project(crane_debug_tools)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# CLI Skill Tester
ament_auto_add_executable(crane_skill_cli
  src/cli_skill_tester.cpp
)

# Simple Web Server (temporary solution)
ament_auto_add_executable(crane_web_server
  src/simple_web_server.cpp
)

# Install launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# Install web files
install(DIRECTORY web DESTINATION share/${PROJECT_NAME})

# Install scripts
install(PROGRAMS scripts/crane_skill DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()