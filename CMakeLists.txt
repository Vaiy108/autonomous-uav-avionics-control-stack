cmake_minimum_required(VERSION 3.20)

project(
    autonomous_uav_avionics_control_stack
    VERSION 0.1.0
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

enable_testing()

# ============================================================
# Embedded avionics demo
# ============================================================

add_executable(
    avionics_demo
    src/application/main.cpp
    src/drivers/imu/SimulatedImuDriver.cpp
    src/drivers/gnss/SimulatedGnssDriver.cpp
    src/drivers/barometer/SimulatedBarometerDriver.cpp
    src/drivers/magnetometer/SimulatedMagnetometerDriver.cpp
    src/middleware/message_bus/LocalMessageBus.cpp
    src/middleware/sensor_manager/SensorManager.cpp
    src/services/health_monitor/SensorHealthMonitor.cpp
    src/platform/simulation/SimulationClock.cpp
    src/platform/simulation/SimulatedUart.cpp
    src/platform/simulation/SimulatedSpiBus.cpp
    src/platform/simulation/SimulatedI2cBus.cpp
)

target_include_directories(
    avionics_demo
    PRIVATE
        ${PROJECT_SOURCE_DIR}/include
)

# ============================================================
# Flight-control library
# ============================================================

add_library(
    flight_control_lib
    src/control/pid_controller.cpp
    src/control/attitude_controller.cpp
    src/control/altitude_controller.cpp
    src/control/position_controller.cpp
    src/actuation/motor_mixer.cpp
    src/actuation/actuator_model.cpp
    src/simulation/quadrotor_dynamics.cpp
    src/simulation/quadrotor_6dof.cpp
)

target_include_directories(
    flight_control_lib
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/src/control
        ${PROJECT_SOURCE_DIR}/src/actuation
        ${PROJECT_SOURCE_DIR}/src/simulation
)

# ============================================================
# Compiler warnings
# ============================================================

if(MSVC)
    target_compile_options(avionics_demo PRIVATE /W4 /permissive-)
    target_compile_options(flight_control_lib PRIVATE /W4 /permissive-)
else()
    target_compile_options(avionics_demo PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(flight_control_lib PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ============================================================
# Examples
# ============================================================

add_executable(
    closed_loop_attitude_demo
    examples/closed_loop_attitude_demo.cpp
)
target_link_libraries(
    closed_loop_attitude_demo
    PRIVATE flight_control_lib
)

add_executable(
    position_hold_6dof_demo
    examples/position_hold_6dof_demo.cpp
)
target_link_libraries(
    position_hold_6dof_demo
    PRIVATE flight_control_lib
)

add_executable(
    closed_loop_6dof_demo
    examples/closed_loop_6dof_demo.cpp
)
target_link_libraries(
    closed_loop_6dof_demo
    PRIVATE flight_control_lib
)

# ============================================================
# Unit tests
# ============================================================

function(add_fc_test TEST_NAME TEST_SOURCE)
    add_executable(
        ${TEST_NAME}
        ${TEST_SOURCE}
    )

    target_link_libraries(
        ${TEST_NAME}
        PRIVATE flight_control_lib
    )

    add_test(
        NAME ${TEST_NAME}
        COMMAND ${TEST_NAME}
    )
endfunction()

add_fc_test(test_pid_controller
    tests/unit/test_pid_controller.cpp)

add_fc_test(test_attitude_controller
    tests/unit/test_attitude_controller.cpp)

add_fc_test(test_position_controller
    tests/unit/test_position_controller.cpp)

add_fc_test(test_motor_mixer
    tests/unit/test_motor_mixer.cpp)

add_fc_test(test_actuator_model
    tests/unit/test_actuator_model.cpp)

add_fc_test(test_quadrotor_dynamics
    tests/unit/test_quadrotor_dynamics.cpp)

add_fc_test(test_quadrotor_6dof
    tests/unit/test_quadrotor_6dof.cpp)
