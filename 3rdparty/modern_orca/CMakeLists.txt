cmake_minimum_required(VERSION 3.16)
project(modern_orca
  VERSION 0.1.0
  DESCRIPTION "Modern C++ ORCA Library with Extensible Constraints"
  LANGUAGES CXX)

# Modern C++ features
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler-specific options
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_options(-g -O0)
  else()
    add_compile_options(-O3 -DNDEBUG)
  endif()
endif()

# Options
option(MODERN_ORCA_BUILD_EXAMPLES "Build example applications" OFF)
option(MODERN_ORCA_BUILD_TESTS "Build unit tests" ON)
option(MODERN_ORCA_BUILD_BENCHMARKS "Build benchmarks" OFF)
option(MODERN_ORCA_ENABLE_OPENMP "Enable OpenMP for parallel processing" ON)

# Find dependencies
find_package(Threads REQUIRED)

if(MODERN_ORCA_ENABLE_OPENMP)
  find_package(OpenMP)
endif()

# Header-only library target
add_library(modern_orca INTERFACE)
add_library(modern_orca::modern_orca ALIAS modern_orca)

target_include_directories(modern_orca
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(modern_orca INTERFACE cxx_std_20)

target_link_libraries(modern_orca
  INTERFACE
    Threads::Threads
    $<$<TARGET_EXISTS:OpenMP::OpenMP_CXX>:OpenMP::OpenMP_CXX>
)

# Examples
if(MODERN_ORCA_BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

# Tests
if(MODERN_ORCA_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

# Benchmarks
if(MODERN_ORCA_BUILD_BENCHMARKS)
  add_subdirectory(benchmarks)
endif()

# Installation
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(TARGETS modern_orca
  EXPORT modern_orca-targets
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  FILES_MATCHING PATTERN "*.hpp"
)

install(EXPORT modern_orca-targets
  FILE modern_orca-targets.cmake
  NAMESPACE modern_orca::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/modern_orca
)

configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modern_orca-config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/modern_orca-config.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/modern_orca
)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/modern_orca-config-version.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/modern_orca-config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/modern_orca-config-version.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/modern_orca
)

# Export from build tree
export(EXPORT modern_orca-targets
  FILE ${CMAKE_CURRENT_BINARY_DIR}/modern_orca-targets.cmake
  NAMESPACE modern_orca::
)
