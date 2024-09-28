cmake_minimum_required(VERSION 3.8)
project(abseil_cpp_vendor)

# find dependencies
find_package(ament_cmake REQUIRED)

include(ExternalProject)
externalproject_add(abseil_cpp_project
  PREFIX ${CMAKE_BINARY_DIR}/abseil_cpp
  GIT_REPOSITORY https://github.com/abseil/abseil-cpp.git
  GIT_TAG master
  TIMEOUT 6000
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}/abseil_cpp/install -DABSL_PROPAGATE_CXX_STD=ON
)

# ヘッダーとライブラリを指定
install(DIRECTORY ${CMAKE_BINARY_DIR}/abseil_cpp/install/include/
        DESTINATION ${CMAKE_INSTALL_PREFIX}/include/abseil_cpp_vendor/absl
)
install(DIRECTORY ${CMAKE_BINARY_DIR}/abseil_cpp/install/lib/
        DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)


# ライブラリフォルダ内のすべての.aおよび.soファイルを取得してエクスポートする
file(GLOB LIB_FILES "${CMAKE_INSTALL_PREFIX}/lib/*.a")


ament_export_include_directories(${CMAKE_INSTALL_PREFIX}/include/abseil_cpp_vendor)
message("LIB_FILES: ${LIB_FILES}")
ament_export_libraries(${LIB_FILES})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
