cmake_minimum_required(VERSION 3.16)
project(modern_orca)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

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
option(MODERN_ORCA_BUILD_TESTS "Build unit tests" ON)
option(MODERN_ORCA_BUILD_BENCHMARKS "Build benchmarks" OFF)
option(MODERN_ORCA_ENABLE_OPENMP "Enable OpenMP for parallel processing" ON)

# Additional dependencies
find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
if(MODERN_ORCA_ENABLE_OPENMP)
  find_package(OpenMP)
endif()

# Header-only library target
ament_auto_add_library(modern_orca INTERFACE)
add_library(modern_orca::modern_orca ALIAS modern_orca)

target_compile_features(modern_orca INTERFACE cxx_std_20)

target_link_libraries(modern_orca
  INTERFACE
    Threads::Threads
    $<$<TARGET_EXISTS:OpenMP::OpenMP_CXX>:OpenMP::OpenMP_CXX>
)

# Tests
if(MODERN_ORCA_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

# Benchmarks
if(MODERN_ORCA_BUILD_BENCHMARKS)
  add_subdirectory(benchmarks)
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
