
cmake_minimum_required(VERSION 3.20)

project(
    embedded_avionics_sensor_platform
    VERSION 0.1.0
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(
    avionics_demo
    src/application/main.cpp
    src/drivers/imu/SimulatedImuDriver.cpp
    src/drivers/gnss/SimulatedGnssDriver.cpp
    src/drivers/barometer/SimulatedBarometerDriver.cpp
    src/drivers/magnetometer/SimulatedMagnetometerDriver.cpp
    src/middleware/message_bus/LocalMessageBus.cpp
)

target_include_directories(
    avionics_demo
    PRIVATE
    ${PROJECT_SOURCE_DIR}/include
)

if(MSVC)
    target_compile_options(avionics_demo PRIVATE /W4 /permissive-)
else()
    target_compile_options(avionics_demo PRIVATE -Wall -Wextra -Wpedantic)
endif()