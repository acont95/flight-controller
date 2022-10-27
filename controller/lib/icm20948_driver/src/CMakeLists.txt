cmake_minimum_required(VERSION 3.20.0)
project(icm20948_driver)

add_library(${PROJECT_NAME} STATIC icm20948_driver.cpp)