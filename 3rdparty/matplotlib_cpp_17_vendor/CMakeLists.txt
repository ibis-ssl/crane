cmake_minimum_required(VERSION 3.12)
project(matplotlib_cpp_17_vendor)


# Default to Release build if not specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()
# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

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

macro(build_matplotlibcpp17)
  set(cmake_commands)
  set(cmake_configure_args -Wno-dev)
  set(cmake_configure_args -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_install)

  # Check if already built or if SKIP_VENDOR_CLONE is set
  if(EXISTS ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_install/include)
    message(STATUS "matplotlib_cpp_17_vendor already built, skipping")
    return()
  endif()

  if(DEFINED ENV{SKIP_VENDOR_CLONE} OR SKIP_VENDOR_CLONE)
    message(STATUS "SKIP_VENDOR_CLONE is set, skipping matplotlib_cpp_17_vendor clone")
    return()
  endif()

  # Check if source already exists to enable offline build
  set(MATPLOTLIB_SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/matplotlib-cpp-ext-prefix/src/matplotlib-cpp-ext)
  if(EXISTS ${MATPLOTLIB_SOURCE_DIR}/.git)
    message(STATUS "既存のmatplotlibcpp17ソースを検出しました。オフラインモードで利用します: ${MATPLOTLIB_SOURCE_DIR}")
    set(UPDATE_DISCONNECTED_FLAG UPDATE_DISCONNECTED 1)
  else()
    message(STATUS "matplotlibcpp17ソースが見つかりません。GitHubからダウンロードします")
    set(UPDATE_DISCONNECTED_FLAG UPDATE_DISCONNECTED 0)
  endif()

  include(ExternalProject)
  externalproject_add(matplotlib-cpp-ext
    GIT_REPOSITORY https://github.com/soblin/matplotlibcpp17.git
    GIT_TAG 4d025b5975b701598fce8487cd8feaf82eb5923f
    GIT_PROGRESS ON
    ${UPDATE_DISCONNECTED_FLAG}
    GIT_SUBMODULES_RECURSE OFF
    TIMEOUT 6000
    ${cmake_commands}
    CMAKE_ARGS
      ${cmake_configure_args}
  )

  # The external project will install to the build folder, but we'll install that on make install.
  install(
    DIRECTORY
      ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_install/include/
    DESTINATION
      ${CMAKE_INSTALL_PREFIX}/include/matplotlibcpp17
  )
endmacro()

build_matplotlibcpp17()
ament_export_include_directories(${CMAKE_INSTALL_PREFIX}/include/matplotlibcpp17)
message(${CMAKE_INSTALL_PREFIX}/include/matplotlibcpp17)
ament_export_libraries(matplotlibcpp17)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
