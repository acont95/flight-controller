cmake_minimum_required(VERSION 3.20.0)

# Pull in pico sdk
include(pico_sdk_import.cmake)

# Pull in FreeRTOS
include(FreeRTOS_Kernel_import.cmake)

project(flight_controller_app C CXX ASM)

# Initialise the Pico SDK
pico_sdk_init()

add_subdirectory(${CMAKE_SOURCE_DIR}/lib/hcsr04_driver)
add_subdirectory(${CMAKE_SOURCE_DIR}/lib/ms5611_driver)
add_subdirectory(${CMAKE_SOURCE_DIR}/lib/icm20948_driver)
add_subdirectory(${CMAKE_SOURCE_DIR}/lib/nmea_parser)


# add_library(microros STATIC IMPORTED)
# set_property(
#     TARGET microros PROPERTY
#     IMPORTED_LOCATION "${CMAKE_SOURCE_DIR}/micro_ros_raspberrypi_pico_sdk/libmicroros/libmicroros.a"
# )

add_executable(main_exec
    ${CMAKE_SOURCE_DIR}/src/main.cpp
)

target_include_directories(
    main_exec
        PRIVATE 
            ${CMAKE_SOURCE_DIR}/include
            ${CMAKE_SOURCE_DIR}/lib/hcsr04_driver/include
            ${CMAKE_SOURCE_DIR}/lib/ms5611_driver/include
            ${CMAKE_SOURCE_DIR}/lib/icm20948_driver/include
            ${CMAKE_SOURCE_DIR}/lib/nmea_parser/include


            # ${CMAKE_SOURCE_DIR}/micro_ros_raspberrypi_pico_sdk/libmicroros/include
            # ${CMAKE_SOURCE_DIR}/micro_ros_raspberrypi_pico_sdk
)

target_link_libraries(main_exec 
    pico_stdlib 
    hardware_spi
    FreeRTOS-Kernel 
    FreeRTOS-Kernel-Heap1
    # microros
    hcsr04_driver
    ms5611_driver
    icm20948_driver
    nmea_parser
)


target_sources(
    main_exec
        PRIVATE 
            # ${PROJECT_SOURCE_DIR}/src/main.cpp
            # ${PROJECT_SOURCE_DIR}/src/baro_manager.cpp 
            # ${PROJECT_SOURCE_DIR}/src/flight_controller.cpp 
            # ${PROJECT_SOURCE_DIR}/src/flight_controller_node.cpp 
            # ${PROJECT_SOURCE_DIR}/src/gps_manager.cpp 
            ${PROJECT_SOURCE_DIR}/src/imu_manager.cpp  
            # ${PROJECT_SOURCE_DIR}/src/test_printer.cpp 
            # ${PROJECT_SOURCE_DIR}/src/main.c 

            ${CMAKE_SOURCE_DIR}/src/pico_gpio_output_impl.cpp 
            ${CMAKE_SOURCE_DIR}/src/pico_sleep_impl.cpp 
            ${CMAKE_SOURCE_DIR}/src/pico_timer_impl.cpp 
            ${CMAKE_SOURCE_DIR}/src/pico_spi_bus_impl.cpp 
            # ${CMAKE_SOURCE_DIR}/micro_ros_raspberrypi_pico_sdk/pico_uart_transport.c
            ${CMAKE_SOURCE_DIR}/src/baro_manager.cpp 
            ${CMAKE_SOURCE_DIR}/src/ultrasonic_distance_manager.cpp 
            # ${CMAKE_SOURCE_DIR}/src/ultrasonic_test.cpp
)

pico_enable_stdio_usb(main_exec 1)
pico_enable_stdio_uart(main_exec 0)
pico_add_extra_outputs(main_exec)