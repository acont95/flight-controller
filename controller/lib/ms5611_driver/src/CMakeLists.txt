cmake_minimum_required(VERSION 3.20.0)
project(ms5611_driver)

add_library(${PROJECT_NAME} STATIC ms5611_driver.cpp)