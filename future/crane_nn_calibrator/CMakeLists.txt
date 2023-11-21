cmake_minimum_required(VERSION 3.5)
project(crane_nn_calibrator)

find_package(ament_cmake        REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
        DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}/
        DESTINATION lib/${PROJECT_NAME})

install(
        DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/config
        ${CMAKE_CURRENT_SOURCE_DIR}/launch
        DESTINATION share/${PROJECT_NAME})

ament_package()
