cmake_minimum_required(VERSION 3.5)
project(crane_planner_plugins)

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

add_definitions("-DBOOST_ALLOW_DEPRECATED_HEADERS")

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp_components REQUIRED)
ament_auto_find_build_dependencies()

# for matplotlib
find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
find_package(PythonLibs 3.0 REQUIRED)
include_directories(${PYTHON3_INCLUDE_DIRS} ${NumPy_INCLUDE_DIRS})

function(add_planner_plugin)
  # <prefix> <options> <one_value_keywords> <multi_value_keywords> <args>...
  cmake_parse_arguments(PARSE_ARGV 0  "PLUGIN" "" "NAME;CLASS_NAME" "")

  message("plugin name = ${PLUGIN_NAME}")
  message("plugin class name = ${PLUGIN_CLASS_NAME}")

  ament_auto_add_library(${PLUGIN_NAME} SHARED src/${PLUGIN_NAME}/${PLUGIN_NAME}.cpp)
  rclcpp_components_register_nodes(${PLUGIN_NAME} ${PLUGIN_CLASS_NAME})
  ament_auto_add_executable(${PLUGIN_NAME}_node src/${PLUGIN_NAME}/${PLUGIN_NAME}_node.cpp)
  target_link_libraries(${PLUGIN_NAME}_node ${PLUGIN_NAME})
  install(TARGETS ${PLUGIN_NAME} ${PLUGIN_NAME}_node
    ARCHIVE DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME}
    RUNTIME DESTINATION lib/${PROJECT_NAME})
endfunction()

add_planner_plugin(
  NAME "ball_placement"
  CLASS_NAME "crane::BallPlacementPlanner"
)

add_planner_plugin(
  NAME "ball_placement_with_skill"
  CLASS_NAME "crane::BallPlacementWithSkillPlanner"
)

add_planner_plugin(
  NAME "receive"
  CLASS_NAME "crane::ReceivePlanner"
)

ament_auto_add_executable(receive_test_node src/receive/test.cpp)
ament_target_dependencies(receive_test_node "Python3")


add_planner_plugin(
  NAME "attacker"
  CLASS_NAME "crane::AttackerPlanner"
)

add_planner_plugin(
  NAME "waiter"
  CLASS_NAME "crane::WaiterPlanner"
)

add_planner_plugin(
  NAME "formation"
  CLASS_NAME "crane::FormationPlanner"
)

add_planner_plugin(
  NAME "goalie"
  CLASS_NAME "crane::GoaliePlanner"
)

add_planner_plugin(
  NAME "tigers_goalie"
  CLASS_NAME "crane::TigersGoaliePlanner"
)

add_planner_plugin(
  NAME "defender"
  CLASS_NAME "crane::DefenderPlanner"
)

add_planner_plugin(
  NAME "kickoff"
  CLASS_NAME "crane::KickOffPlanner"
)

add_planner_plugin(
    NAME "marker"
    CLASS_NAME "crane::MarkerPlanner"
)

add_planner_plugin(
  NAME "template"
  CLASS_NAME "crane::TemplatePlanner"
)

pluginlib_export_plugin_description_file(crane_planner_base plugins.xml)

if (BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif ()

ament_auto_package()
