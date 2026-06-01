cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME rtos_cmake_cubemx)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

set(MPU6050_DRV_DIR ${CMAKE_SOURCE_DIR}/Drivers/mpu6050)
set(SERVO_DRV_DIR ${CMAKE_SOURCE_DIR}/Drivers/servo)
set(APP_DIR ${CMAKE_SOURCE_DIR}/App)
set(PLATFORM_DIR ${CMAKE_SOURCE_DIR}/Platform)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${MPU6050_DRV_DIR}/mpu6050_driver.c
    ${MPU6050_DRV_DIR}/mpu6050_stm32_hal_port.c
    ${SERVO_DRV_DIR}/servo_driver.c
    ${SERVO_DRV_DIR}/servo_stm32_hal_port.c
    ${APP_DIR}/SensorAcq/global_data_space.c
    ${APP_DIR}/SensorAcq/sensor_manager.c
    ${APP_DIR}/Navigation/navigation_subsystem.c
    ${APP_DIR}/FlightManagementControl/flight_control.c
    ${APP_DIR}/Telemetry/telemetry_task.c
    ${PLATFORM_DIR}/STM32/mpu6050_ddi_adapter.c
    ${PLATFORM_DIR}/STM32/app_platform_port.c
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/Drivers
    ${MPU6050_DRV_DIR}
    ${SERVO_DRV_DIR}
    ${APP_DIR}/SensorAcq
    ${APP_DIR}/Navigation
    ${APP_DIR}/FlightManagementControl
    ${APP_DIR}/Telemetry
    ${PLATFORM_DIR}/STM32
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

option(BUILD_HOST_TESTS "Build host-side GoogleTest scaffolding" OFF)
if(BUILD_HOST_TESTS)
    add_subdirectory(tests/host)
endif()
