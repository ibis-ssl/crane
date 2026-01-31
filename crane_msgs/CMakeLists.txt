cmake_minimum_required(VERSION 3.12)
project(crane_msgs)


# Default to Release build if not specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
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

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Collect interface files automatically
file(GLOB msg_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} CONFIGURE_DEPENDS "msg/*.msg")
file(GLOB action_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} CONFIGURE_DEPENDS "action/*.action")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${action_files}
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs robocup_ssl_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
