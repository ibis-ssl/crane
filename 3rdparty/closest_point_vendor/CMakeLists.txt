cmake_minimum_required(VERSION 3.5)
project(closest_point_vendor)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

macro(build_closest_point)
  set(cmake_commands)
  set(cmake_configure_args -Wno-dev;-DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_install)
  include(ExternalProject)
  set(closest_point_source_path ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_install/src/closest_point-ext)
  set(closest_point_cmake_file_path ${closest_point_source_path}/CMakeLists.txt)
  externalproject_add(closest_point-ext
    PREFIX ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_install
    INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
    GIT_REPOSITORY https://github.com/artemp/closest_point.git
    GIT_TAG master
    TIMEOUT 6000
    PATCH_COMMAND
      COMMAND echo cmake_minimum_required( VERSION 3.5 ) > ${closest_point_cmake_file_path}
      COMMAND echo project( closest_point_vendor ) >> ${closest_point_cmake_file_path}
      COMMAND echo install( DIRECTORY boost DESTINATION include FILES_MATCHING PATTERN "*.hpp*" ) >> ${closest_point_cmake_file_path}
    CMAKE_ARGS
      ${cmake_configure_args}
    ${cmake_commands}
    CMAKE_ARGS
      ${cmake_configure_args}
    )
    install(
      DIRECTORY
        ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_install/
      DESTINATION
        ${CMAKE_INSTALL_PREFIX}
  )
endmacro()

build_closest_point()
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
