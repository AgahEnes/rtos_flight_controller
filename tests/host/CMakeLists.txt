cmake_minimum_required(VERSION 3.22)

project(host_tests LANGUAGES C CXX)

include(CTest)
enable_testing()

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
)
FetchContent_MakeAvailable(googletest)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(--coverage -O0 -g)
  add_link_options(--coverage)
endif()

add_library(app_test_objs STATIC
  ../../App/SensorAcq/global_data_space.c
  ../../App/SensorAcq/sensor_manager.c
  ../../App/Actuator/actuator_manager.c
  ../../App/Navigation/navigation_subsystem.c
  ../../App/FlightManagementControl/flight_control.c
  ../../App/Telemetry/telemetry_task.c
)
target_include_directories(app_test_objs PUBLIC
  ../../App/SensorAcq
  ../../App/Actuator
  ../../App/Navigation
  ../../App/FlightManagementControl
  ../../App/Telemetry
  ../../Drivers/mpu6050
  .
)

add_library(driver_test_objs STATIC
  ../../Drivers/mpu6050/mpu6050_driver.c
  mocks/mock_bus.c
  mocks/mock_lock_time.c
)
target_include_directories(driver_test_objs PUBLIC
  ../../Drivers
  ../../Drivers/mpu6050
  .
)

add_library(servo_driver_test_objs STATIC
  ../../Drivers/servo/servo_driver.c
)
target_include_directories(servo_driver_test_objs PUBLIC
  ../../Drivers
  ../../Drivers/servo
  .
)

add_executable(app_unit_tests
  sensor_acq_test.cpp
)
target_link_libraries(app_unit_tests PRIVATE app_test_objs GTest::gtest_main m)

add_executable(nav_unit_tests
  navigation_subsystem_test.cpp
)
target_link_libraries(nav_unit_tests PRIVATE app_test_objs GTest::gtest_main m)

add_executable(actuator_unit_tests
  actuator_test.cpp
)
target_link_libraries(actuator_unit_tests PRIVATE app_test_objs GTest::gtest_main m)

add_executable(flight_control_unit_tests
  flight_control_test.cpp
)
target_link_libraries(flight_control_unit_tests PRIVATE app_test_objs GTest::gtest_main m)

add_executable(mpu_driver_tests
  mpu6050_driver_test.cpp
)
target_link_libraries(mpu_driver_tests PRIVATE driver_test_objs GTest::gtest_main m)

add_executable(servo_driver_tests
  servo_driver_test.cpp
)
target_link_libraries(servo_driver_tests PRIVATE servo_driver_test_objs GTest::gtest_main)

add_test(NAME app_unit_tests COMMAND app_unit_tests)
add_test(NAME nav_unit_tests COMMAND nav_unit_tests)
add_test(NAME actuator_unit_tests COMMAND actuator_unit_tests)
add_test(NAME flight_control_unit_tests COMMAND flight_control_unit_tests)
add_test(NAME mpu_driver_tests COMMAND mpu_driver_tests)
add_test(NAME servo_driver_tests COMMAND servo_driver_tests)
