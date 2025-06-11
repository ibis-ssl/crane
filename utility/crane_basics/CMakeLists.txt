cmake_minimum_required(VERSION 3.28) # Increased for better module support
project(crane_basics)

# Default to C++20 and require it
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF) # Recommended

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g -fmodules-ts)
  # Clang might need: -stdlib=libc++ if not default
endif()
# Add MSVC specific flags here if necessary (e.g., /EHsc /MD /std:c++20)

add_definitions("-DBOOST_ALLOW_DEPRECATED_HEADERS") # Keep if Boost headers require it

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
# Let ament_auto handle finding build dependencies for now
ament_auto_find_build_dependencies()

# Enable CMake's C++ module scanning feature
set(CMAKE_CXX_SCAN_FOR_MODULES ON)

# List of all files that constitute the crane_basics module interface or its partitions
# This list will grow as more files are converted.
set(CRANE_BASICS_MODULE_SOURCES
  "include/crane_basics/crane_basics.cppm"
  "include/crane_basics/boost_geometry.hpp"
  "include/crane_basics/ball_info.hpp"
  "include/crane_basics/geometry_operations.hpp"
  "include/crane_basics/time.hpp"
  # IMPORTANT: Files like capsule.hpp, circle.hpp, eigen_adapter.hpp
  # are currently #included by boost_geometry.hpp. They MUST be converted
  # to partitions and added here, and boost_geometry.hpp updated to import them.
  # This will be handled in a subsequent step.
)

# Add the library target for the module.
# Modules typically produce compiled artifacts, so STATIC is more appropriate than INTERFACE.
add_library(crane_basics_module STATIC ${CRANE_BASICS_MODULE_SOURCES})
set_target_properties(crane_basics_module PROPERTIES CXX_MODULE_PRIMARY_INTERFACE "include/crane_basics/crane_basics.cppm")

# Public include directories for consumers of this module.
# They need to find the source .cppm/.hpp module files.
target_include_directories(crane_basics_module
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include> # This makes "include" available in install space
    ${EIGEN3_INCLUDE_DIR} # Eigen headers
)

# Link against Eigen3
target_link_libraries(crane_basics_module PUBLIC Eigen3::Eigen)

# Explicitly declare dependencies found by ament_auto_find_build_dependencies
# This helps ensure proper linkage for things like rclcpp, std_msgs if they are dependencies
target_link_libraries(crane_basics_module PUBLIC ${ament_cmake_auto_FOUND_BUILD_DEPENDS_TARGETS})


# The line ament_export_include_directories(include ${EIGEN3_INCLUDE_DIR}) is removed.

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_auto_add_gtest(test_geometry test/test.cpp)
  # The test needs to link against the module library
  target_link_libraries(test_geometry PRIVATE crane_basics_module)
endif()

# Install module source files (needed by compilers of dependent packages)
install(
  DIRECTORY include/crane_basics/
  DESTINATION include/crane_basics
  FILES_MATCHING PATTERN "*.cppm" PATTERN "*.hpp"
)

# Install the compiled module library and export the target
install(
  TARGETS crane_basics_module
  EXPORT crane_basics_module_targets # This name is used by ament_export_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_auto_package()
# Consider replacing ament_auto_package() later with:
# ament_export_targets(crane_basics_module_targets HAS_LIBRARY_TARGET)
# ament_export_dependencies(Eigen3 rclcpp std_msgs) # List all exported dependencies
