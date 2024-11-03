cmake_minimum_required(VERSION 3.5)
project(rvo2_vendor)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -g)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

set(cmake_commands)
set(cmake_configure_args -Wno-dev)
set(cmake_configure_args -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_install)

include(ExternalProject)
externalproject_add(rvo2-ext
    GIT_REPOSITORY https://github.com/snape/RVO2.git
    GIT_TAG af26bedf27a84ffffb59beea996ffe2531ddc789
    TIMEOUT 6000
    ${cmake_commands}
    CMAKE_ARGS
    ${cmake_configure_args}
)

# The external project will install to the build folder, but we'll install that on make install.
install(
    DIRECTORY
        ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_install/include/
    DESTINATION
        ${CMAKE_INSTALL_PREFIX}/include/rvo2_vendor
)

install(
    DIRECTORY
        ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_install/lib/
    DESTINATION
        ${CMAKE_INSTALL_PREFIX}/lib
)


ament_export_include_directories(include)
ament_export_libraries(RVO)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
