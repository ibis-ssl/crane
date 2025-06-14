cmake_minimum_required(VERSION 3.5)
project(crane_basics)

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
ament_auto_find_build_dependencies()

ament_export_include_directories(
  include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Individual test files
  ament_auto_add_gtest(test_geometry test/test_geometry.cpp)
  target_include_directories(test_geometry PUBLIC include)
  target_include_directories(test_geometry PUBLIC ${closest_point_vendor_INCLUDE_DIRS})

  ament_auto_add_gtest(test_travel_time test/test_travel_time.cpp)
  target_include_directories(test_travel_time PUBLIC include)
  target_include_directories(test_travel_time PUBLIC ${closest_point_vendor_INCLUDE_DIRS})

  ament_auto_add_gtest(test_pid_controller test/test_pid_controller.cpp)
  target_include_directories(test_pid_controller PUBLIC include)
  target_include_directories(test_pid_controller PUBLIC ${closest_point_vendor_INCLUDE_DIRS})

  ament_auto_add_gtest(test_interval test/test_interval.cpp)
  target_include_directories(test_interval PUBLIC include)
  target_include_directories(test_interval PUBLIC ${closest_point_vendor_INCLUDE_DIRS})

  ament_auto_add_gtest(test_robot_info test/test_robot_info.cpp)
  target_include_directories(test_robot_info PUBLIC include)
  target_include_directories(test_robot_info PUBLIC ${closest_point_vendor_INCLUDE_DIRS})

  ament_auto_add_gtest(test_ball_info test/test_ball_info.cpp)
  target_include_directories(test_ball_info PUBLIC include)
  target_include_directories(test_ball_info PUBLIC ${closest_point_vendor_INCLUDE_DIRS})

  ament_auto_add_gtest(test_time test/test_time.cpp)
  target_include_directories(test_time PUBLIC include)
  target_include_directories(test_time PUBLIC ${closest_point_vendor_INCLUDE_DIRS})

  ament_auto_add_gtest(test_vector_operations test/test_vector_operations.cpp)
  target_include_directories(test_vector_operations PUBLIC include)
  target_include_directories(test_vector_operations PUBLIC ${closest_point_vendor_INCLUDE_DIRS})

  ament_auto_add_gtest(test_ball_model test/test_ball_model.cpp)
  target_include_directories(test_ball_model PUBLIC include)
  target_link_directories(test_ball_model PUBLIC ${closest_point_vendor_INCLUDE_DIRS})
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
