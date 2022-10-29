cmake_minimum_required(VERSION 3.20.0)


list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/modules/libmicroros)

# set(ZEPHYR_EXTRA_MODULES
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/libmicroros
# )

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(flight_controller_app)


# add_subdirectory(${PROJECT_SOURCE_DIR}/lib/hcsr04_driver)
# add_subdirectory(${PROJECT_SOURCE_DIR}/lib/icm20948_driver)
# add_subdirectory(${PROJECT_SOURCE_DIR}/lib/ms5611_driver)
# add_subdirectory(${PROJECT_SOURCE_DIR}/lib/hcsr04_driver)

# add_subdirectory(${PROJECT_SOURCE_DIR}/lib/hcsr04_driver)




target_include_directories(
    app
        PRIVATE 
            ${PROJECT_SOURCE_DIR}/include
)



# link_directories(${PROJECT_SOURCE_DIR}/lib/hcsr04_driver)

target_sources(
    app
        PRIVATE 
            # ${PROJECT_SOURCE_DIR}/src/main.cpp
            # ${PROJECT_SOURCE_DIR}/src/baro_manager.cpp 
            # ${PROJECT_SOURCE_DIR}/src/flight_controller.cpp 
            # ${PROJECT_SOURCE_DIR}/src/flight_controller_node.cpp 
            # ${PROJECT_SOURCE_DIR}/src/gps_manager.cpp 
            # ${PROJECT_SOURCE_DIR}/src/imu_manager.cpp  
            # ${PROJECT_SOURCE_DIR}/src/test_printer.cpp 
            ${PROJECT_SOURCE_DIR}/src/ultrasonic_distance_manager.cpp 
            ${PROJECT_SOURCE_DIR}/src/hcsr04_us.cpp 
)

# target_sources(app PRIVATE src/main.c)