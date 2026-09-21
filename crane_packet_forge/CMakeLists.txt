cmake_minimum_required(VERSION 3.12)
project(crane_packet_forge)

# 既定を Release にする。他パッケージと同じ条件でローカルビルドするため。
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(crane_sender REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# ros2 run crane_packet_forge crane-forge ... で呼べるようにする
install(PROGRAMS scripts/crane-forge DESTINATION lib/${PROJECT_NAME})

# GUI の静的ファイルと、layout.py を再生成するツール
install(DIRECTORY web DESTINATION share/${PROJECT_NAME})
install(DIRECTORY tools DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  # C++ 側のシリアライザが golden_vectors.json を再現することを確かめる。
  # Python 側の test_assemble_golden.py が同じ JSON と突き合わせるので、
  # 片方だけずれたらどちらかのテストが落ちる。
  ament_add_gtest(test_golden_vectors test/test_golden_vectors.cpp)
  if(TARGET test_golden_vectors)
    ament_target_dependencies(test_golden_vectors crane_sender)
    target_compile_definitions(test_golden_vectors
      PRIVATE GOLDEN_VECTORS_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/golden_vectors.json")
  endif()

  # golden_vectors.json の再生成用。中身は C++ 側が正本。
  add_executable(dump_golden tools/dump_golden.cpp)
  ament_target_dependencies(dump_golden crane_sender)
  install(TARGETS dump_golden DESTINATION lib/${PROJECT_NAME})

  ament_add_pytest_test(${PROJECT_NAME}_pytest test
    TIMEOUT 120
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

ament_package()
