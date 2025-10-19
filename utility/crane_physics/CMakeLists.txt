cmake_minimum_required(VERSION 3.5)
project(crane_physics)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif()

add_definitions("-DBOOST_ALLOW_DEPRECATED_HEADERS")

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/ball_physics_model.cpp
  src/ball_info.cpp
  src/kicker_model.cpp
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  yaml-cpp
)

ament_export_include_directories(
  include
)
ament_export_libraries(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Physics tests
  ament_auto_add_gtest(test_travel_time test/test_travel_time.cpp)
  target_include_directories(test_travel_time PUBLIC include)
  target_link_libraries(test_travel_time Eigen3::Eigen)

  ament_auto_add_gtest(test_pid_controller test/test_pid_controller.cpp)
  target_include_directories(test_pid_controller PUBLIC include)
  target_link_libraries(test_pid_controller Eigen3::Eigen)

  ament_auto_add_gtest(test_robot_info test/test_robot_info.cpp)
  target_include_directories(test_robot_info PUBLIC include)
  target_link_libraries(test_robot_info Eigen3::Eigen)

  ament_auto_add_gtest(test_ball_info test/test_ball_info.cpp)
  target_include_directories(test_ball_info PUBLIC include)
  target_link_libraries(test_ball_info Eigen3::Eigen)

  ament_auto_add_gtest(test_ball_model test/test_ball_model.cpp)
  target_include_directories(test_ball_model PUBLIC include)
  target_link_libraries(test_ball_model Eigen3::Eigen)

  ament_auto_add_gtest(test_ball_msg_conversion test/test_ball_msg_conversion.cpp)
  target_include_directories(test_ball_msg_conversion PUBLIC include)
  target_link_libraries(test_ball_msg_conversion Eigen3::Eigen)

  ament_auto_add_gtest(test_kicker_model test/test_kicker_model.cpp src/kicker_model.cpp)
  target_include_directories(test_kicker_model PUBLIC include)
  target_link_libraries(test_kicker_model Eigen3::Eigen)
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
