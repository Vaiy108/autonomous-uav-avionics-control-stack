#include "position_controller.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

using flight_control::PIDConfig;
using flight_control::Position2D;
using flight_control::PositionController;
using flight_control::PositionControllerConfig;

namespace {

bool nearlyEqual(double a, double b, double tol = 1e-9) {
    return std::abs(a - b) < tol;
}

PositionController makeController() {
    PositionControllerConfig cfg;

    cfg.gravity = 9.81;
    cfg.max_tilt_rad = 0.35;

    cfg.x_pid.kp = 1.0;
    cfg.x_pid.ki = 0.0;
    cfg.x_pid.kd = 0.0;
    cfg.x_pid.output_min = -5.0;
    cfg.x_pid.output_max = 5.0;

    cfg.y_pid = cfg.x_pid;

    return PositionController(cfg);
}

void testZeroPositionError() {
    auto controller = makeController();

    Position2D setpoint{};
    Position2D measurement{};

    const auto output =
        controller.update(
            setpoint,
            measurement,
            0.01);

    assert(nearlyEqual(output.roll, 0.0));
    assert(nearlyEqual(output.pitch, 0.0));
}

void testPositiveXCommand() {
    auto controller = makeController();

    Position2D setpoint;
    setpoint.x = 1.0;

    Position2D measurement{};

    const auto output =
        controller.update(
            setpoint,
            measurement,
            0.01);

    // Positive X acceleration requires
    // positive pitch in our convention.
    assert(output.pitch > 0.0);
    assert(nearlyEqual(output.roll, 0.0));
}

void testPositiveYCommand() {
    auto controller = makeController();

    Position2D setpoint;
    setpoint.y = 1.0;

    Position2D measurement{};

    const auto output =
        controller.update(
            setpoint,
            measurement,
            0.01);

    // Current 6-DOF convention:
    // positive Y acceleration requires negative roll.
    assert(output.roll < 0.0);
    assert(nearlyEqual(output.pitch, 0.0));
}

void testTiltLimit() {
    auto controller = makeController();

    Position2D setpoint;
    setpoint.x = 100.0;
    setpoint.y = 100.0;

    Position2D measurement{};

    const auto output =
        controller.update(
            setpoint,
            measurement,
            0.01);

    assert(std::abs(output.pitch) <= 0.35);
    assert(std::abs(output.roll) <= 0.35);
}

void testInvalidInputFailsSafe() {
    auto controller = makeController();

    Position2D setpoint;
    setpoint.x =
        std::numeric_limits<double>::quiet_NaN();

    Position2D measurement{};

    const auto output =
        controller.update(
            setpoint,
            measurement,
            0.01);

    assert(nearlyEqual(output.roll, 0.0));
    assert(nearlyEqual(output.pitch, 0.0));
}

void testReset() {
    PositionControllerConfig cfg;

    cfg.gravity = 9.81;
    cfg.max_tilt_rad = 0.35;

    cfg.x_pid.kp = 0.5;
    cfg.x_pid.ki = 0.2;
    cfg.x_pid.kd = 0.0;
    cfg.x_pid.output_min = -5.0;
    cfg.x_pid.output_max = 5.0;

    cfg.y_pid = cfg.x_pid;

    PositionController controller(cfg);

    Position2D setpoint;
    setpoint.x = 1.0;

    Position2D measurement{};

    controller.update(
        setpoint,
        measurement,
        0.01);

    controller.reset();

    Position2D zero_setpoint{};

    const auto output =
        controller.update(
            zero_setpoint,
            measurement,
            0.01);

    assert(nearlyEqual(output.roll, 0.0));
    assert(nearlyEqual(output.pitch, 0.0));
}

}  // namespace

int main() {
    testZeroPositionError();
    testPositiveXCommand();
    testPositiveYCommand();
    testTiltLimit();
    testInvalidInputFailsSafe();
    testReset();

    std::cout
        << "All position controller tests passed.\n";

    return 0;
}
