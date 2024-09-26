cmake_minimum_required(VERSION 3.8)
project(osqp_cpp_vendor)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

macro(build_osqp_cpp)
  set(cmake_commands)
  set(cmake_configure_args -Wno-dev)
  set(cmake_configure_args -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_install)

  include(ExternalProject)
  externalproject_add(osqp-cpp-ext
    GIT_REPOSITORY https://github.com/google/osqp-cpp.git
    TIMEOUT 6000
    ${cmake_commands}
    CMAKE_ARGS
      ${cmake_configure_args}
  )

  install(
    DIRECTORY
      ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_install/include/
    DESTINATION
      ${CMAKE_INSTALL_PREFIX}/include/osqp-cpp
  )
  install(
    DIRECTORY
      ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_install/lib/
    DESTINATION
      ${CMAKE_INSTALL_PREFIX}/lib/osqp-cpp
  )
endmacro()

build_osqp_cpp()
ament_export_include_directories(${CMAKE_INSTALL_PREFIX}/include/osqp-cpp)
#message(${CMAKE_INSTALL_PREFIX}/include/matplotlibcpp17)
ament_export_libraries(osqp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
