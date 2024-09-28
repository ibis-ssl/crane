cmake_minimum_required(VERSION 3.8)
project(osqp_cpp_vendor)

# find dependencies
find_package(ament_cmake REQUIRED)

# オプション設定: インターネットを使用するかどうか
option(USE_INTERNET "Use internet for downloading dependencies" ON)

include(ExternalProject)

# ローカルキャッシュのディレクトリ
set(CACHED_SOURCE_DIR "/path/to/local/osqp-cpp") # 事前にダウンロードしたソースのディレクトリ


if(USE_INTERNET)
  message(STATUS "Using internet to fetch osqp-cpp")
  externalproject_add(osqp_cpp_project
          PREFIX ${CMAKE_BINARY_DIR}/osqp_cpp
          GIT_REPOSITORY https://github.com/google/osqp-cpp.git
          GIT_TAG master
          TIMEOUT 6000
          CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}/osqp_cpp/install -DABSL_PROPAGATE_CXX_STD=ON
  )
else()
  message(STATUS "Using local cache for osqp-cpp")
  externalproject_add(osqp_cpp_project
          PREFIX ${CMAKE_BINARY_DIR}/osqp_cpp
          SOURCE_DIR ${CACHED_SOURCE_DIR}  # ローカルキャッシュを利用
          CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}/osqp_cpp/install -DABSL_PROPAGATE_CXX_STD=ON
  )
endif()

# ヘッダーとライブラリを指定
install(DIRECTORY ${CMAKE_BINARY_DIR}/osqp_cpp/install/include/
        DESTINATION ${CMAKE_INSTALL_PREFIX}/include/osqp-cpp/osqp-cpp
)
install(DIRECTORY ${CMAKE_BINARY_DIR}/osqp_cpp/install/lib/
        DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)
install(FILES ${CMAKE_BINARY_DIR}/osqp_cpp/src/osqp_cpp_project/include/osqp++.h
        DESTINATION ${CMAKE_INSTALL_PREFIX}/include/osqp-cpp/osqp-cpp)
install(FILES ${CMAKE_BINARY_DIR}/osqp_cpp/src/osqp_cpp_project-build/libosqp-cpp.a
        DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

ament_export_include_directories(${CMAKE_INSTALL_PREFIX}/include/osqp-cpp)
ament_export_libraries(osqp-cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
