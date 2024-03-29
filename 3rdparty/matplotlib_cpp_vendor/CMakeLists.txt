cmake_minimum_required(VERSION 3.8)
project(matplotlib_cpp_vendor)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

macro(build_matplotlib)
  set(cmake_commands)
  set(cmake_configure_args -Wno-dev)
  set(cmake_configure_args -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_install)

  include(ExternalProject)
  externalproject_add(matplotlib-cpp-ext
    GIT_REPOSITORY https://github.com/lava/matplotlib-cpp.git
    GIT_TAG ef0383f1315d32e0156335e10b82e90b334f6d9f
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
      ${CMAKE_INSTALL_PREFIX}/include/matplotlib_cpp/matplotlib_cpp
  )
endmacro()

build_matplotlib()
ament_export_include_directories(${CMAKE_INSTALL_PREFIX}/include/matplotlib_cpp)
message(${CMAKE_INSTALL_PREFIX}/include/matplotlib_cpp)
ament_export_libraries(matplotlib_cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
