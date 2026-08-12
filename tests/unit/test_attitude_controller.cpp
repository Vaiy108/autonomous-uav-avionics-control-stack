#include "attitude_controller.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

using flight_control::Attitude;
using flight_control::AttitudeController;
using flight_control::AttitudeControllerConfig;
using flight_control::PIDConfig;

namespace {

bool nearlyEqual(double a, double b, double tol = 1e-9) {
    return std::abs(a - b) < tol;
}

PIDConfig makePIDConfig() {
    PIDConfig cfg;

    cfg.kp = 1.0;
    cfg.ki = 0.0;
    cfg.kd = 0.0;

    cfg.output_min = -1.0;
    cfg.output_max = 1.0;

    return cfg;
}

AttitudeController makeController() {
    AttitudeControllerConfig cfg;

    cfg.roll = makePIDConfig();
    cfg.pitch = makePIDConfig();
    cfg.yaw = makePIDConfig();

    return AttitudeController(cfg);
}

void testZeroError() {
    auto controller = makeController();

    Attitude setpoint{};
    Attitude measurement{};

    const auto command =
        controller.update(setpoint, measurement, 0.01);

    assert(nearlyEqual(command.roll, 0.0));
    assert(nearlyEqual(command.pitch, 0.0));
    assert(nearlyEqual(command.yaw, 0.0));
}

void testRollCommand() {
    auto controller = makeController();

    Attitude setpoint{};
    setpoint.roll = 0.2;

    Attitude measurement{};

    const auto command =
        controller.update(setpoint, measurement, 0.01);

    assert(nearlyEqual(command.roll, 0.2));
    assert(nearlyEqual(command.pitch, 0.0));
    assert(nearlyEqual(command.yaw, 0.0));
}

void testIndependentAxes() {
    auto controller = makeController();

    Attitude setpoint{};
    setpoint.roll = 0.1;
    setpoint.pitch = -0.2;
    setpoint.yaw = 0.3;

    Attitude measurement{};

    const auto command =
        controller.update(setpoint, measurement, 0.01);

    assert(nearlyEqual(command.roll, 0.1));
    assert(nearlyEqual(command.pitch, -0.2));
    assert(nearlyEqual(command.yaw, 0.3));
}

void testYawAngleWrapping() {
    auto controller = makeController();

    constexpr double pi =
        3.14159265358979323846;

    Attitude setpoint{};
    Attitude measurement{};

    setpoint.yaw = -pi + 0.1;
    measurement.yaw = pi - 0.1;

    const auto command =
        controller.update(setpoint, measurement, 0.01);

    // Shortest angular error is +0.2 rad,
    // not approximately -2*pi.
    assert(nearlyEqual(command.yaw, 0.2, 1e-8));
}

void testOutputSaturation() {
    auto controller = makeController();

    Attitude setpoint{};
    setpoint.roll = 10.0;

    Attitude measurement{};

    const auto command =
        controller.update(setpoint, measurement, 0.01);

    assert(command.roll <= 1.0);
    assert(command.roll >= -1.0);
}

}  // namespace

int main() {
    testZeroError();
    testRollCommand();
    testIndependentAxes();
    testYawAngleWrapping();
    testOutputSaturation();

    std::cout
        << "All attitude controller tests passed.\n";

    return 0;
}
