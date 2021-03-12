cmake_minimum_required(VERSION 3.5)
project(crane_msg_wrappers)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# for FastRTPS error
set(CMAKE_PREFIX_PATH /opt/ros/foxy/)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(PkgConfig)
pkg_check_modules(BFL REQUIRED orocos-bfl)
include_directories(${BFL_INCLUDE_DIRS})
link_directories(${BFL_LIBRARY_DIRS})
link_libraries(${BFL_LIBRARIES})

# find dependencies
find_package(ament_cmake_auto REQUIRED)
# find_package(ament_index_cpp REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(nav_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(crane_msgs REQUIRED)
# find_package(crane_3rdparty REQUIRED)
# find_package(crane_geometry REQUIRED)
# find_package(Eigen3 REQUIRED)

set(requirement_dependencies
      "rclcpp"
      "std_msgs"
      "geometry_msgs"
      "nav_msgs"
      "crane_msgs"
      "crane_3rdparty"
      "crane_geometry"
      "Eigen3"
)

ament_auto_find_build_dependencies(REQUIRED ${requirement_dependencies})

include_directories(include)


ament_auto_add_library(${PROJECT_NAME}
        src/geometry_wrapper.cpp)


# add_library(crane_msg_wrappers SHARED
#         src/geometry_wrapper.cpp)



# ament_export_libraries(crane_msg_wrappers)
# ament_export_interfaces(export_crane_msg_wrappers HAS_LIBRARY_TARGET)

ament_target_dependencies(${PROJECT_NAME} ${requirement_dependencies})
# ament_target_dependencies(crane_msg_wrappers
#         "rclcpp"
#         "std_msgs"
#         "geometry_msgs"
#         "nav_msgs"
#         "crane_msgs"
#         "crane_3rdparty"
#         "crane_geometry"
#         "Eigen3"
#         )



# install(TARGETS crane_msg_wrappers
#         EXPORT export_crane_msg_wrappers
#         ARCHIVE DESTINATION lib
#         LIBRARY DESTINATION lib)

# Install header files
# install(
#   DIRECTORY "include/"
#   DESTINATION include
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ament_export_include_directories(
#   include
#   crane_geometry/include
# )
ament_auto_package()
