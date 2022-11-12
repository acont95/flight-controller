cmake_minimum_required(VERSION 3.20.0)
project(icm20948_driver)

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/include
)

add_library(${PROJECT_NAME} STATIC ${PROJECT_SOURCE_DIR}/src/icm20948_imu.cpp)