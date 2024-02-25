cmake_minimum_required(VERSION 3.14)
project(crane_simple_ai)

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -g)
endif ()

add_definitions("-DBOOST_ALLOW_DEPRECATED_HEADERS")

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(Qt5 COMPONENTS Core Gui Widgets Network REQUIRED)
ament_auto_find_build_dependencies()

# Instruct CMake to run moc automatically when needed
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Include directories for Qt
include_directories(${Qt5Widgets_INCLUDES})
add_definitions(${Qt5Widgets_DEFINITIONS})

# Sources and headers
set(SOURCES
        src/simple_ai_node.cpp
        src/crane_commander.cpp
)

set(HEADERS
        include/crane_commander.hpp
)

# Create the executable
ament_auto_add_executable(${PROJECT_NAME}
        ${SOURCES} ${HEADERS}
)

# Use the Widgets module from Qt 5
target_link_libraries(${PROJECT_NAME} Qt5::Core Qt5::Gui Qt5::Widgets)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_auto_package()
