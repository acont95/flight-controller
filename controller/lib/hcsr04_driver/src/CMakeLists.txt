cmake_minimum_required(VERSION 3.20.0)
project(hcsr04_driver)

add_library(${PROJECT_NAME} STATIC hcsr04_us.cpp)