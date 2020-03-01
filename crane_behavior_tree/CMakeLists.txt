cmake_minimum_required(VERSION 3.5)
project(crane_behavior_tree)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(crane_msgs REQUIRED)
find_package(behaviortree_cpp_v3_ibis REQUIRED)

 include_directories(include
         ${CMAKE_CURRENT_BINARY_DIR}
         )
#include_directories(${CMAKE_CURRENT_BINARY_DIR})

set(dependencies
        "rclcpp"
        "std_msgs"
        "crane_msgs"
        "behaviortree_cpp_v3_ibis"
)

set(library_name ${PROJECT_NAME})

add_library(example_action SHARED plugins/action/example_action.cpp)
list(APPEND plugin_libs example_action)

#add_library(<bt_node_name> SHARED <bt_node_name>.cpp)
#list(APPEND plugin_libs <bt_node_name>)

foreach(plugin ${plugin_libs})
  ament_target_dependencies(${plugin} ${dependencies})
  target_compile_definitions(${plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(TARGETS
                ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
        FILES include/${PROJECT_NAME}/world_model.hpp
        DESTINATION include/${PROJECT_NAME})

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

ament_export_libraries(
  ${library_name}
  ${plugin_libs}
)

ament_export_dependencies(${dependencies})

ament_package()
