cmake_minimum_required(VERSION 3.20.0)
project(hcsr04_driver)

include_directories(${CMAKE_SOURCE_DIR}/include)

add_library(${PROJECT_NAME} STATIC ${PROJECT_SOURCE_DIR}/src/hcsr04_us.cpp)