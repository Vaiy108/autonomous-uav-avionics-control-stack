#include "altitude_controller.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

using flight_control::AltitudeController;
using flight_control::AltitudeControllerConfig;
using flight_control::PIDConfig;

namespace {

bool nearlyEqual(double a, double b, double tol = 1e-9) {
    return std::abs(a - b) < tol;
}

AltitudeController makeController() {
    AltitudeControllerConfig cfg;

    cfg.hover_collective = 0.5;
    cfg.collective_min = 0.0;
    cfg.collective_max = 1.0;

    cfg.altitude_pid.kp = 0.2;
    cfg.altitude_pid.ki = 0.05;
    cfg.altitude_pid.kd = 0.0;

    cfg.altitude_pid.output_min = -0.3;
    cfg.altitude_pid.output_max = 0.3;

    cfg.altitude_pid.integral_min = -1.0;
    cfg.altitude_pid.integral_max = 1.0;

    return AltitudeController(cfg);
}

void testZeroAltitudeError() {
    auto controller = makeController();

    const double collective =
        controller.update(1.0, 1.0, 0.01);

    assert(nearlyEqual(collective, 0.5));
}

void testBelowSetpointIncreasesThrust() {
    auto controller = makeController();

    const double collective =
        controller.update(1.0, 0.5, 0.01);

    assert(collective > 0.5);
}

void testAboveSetpointReducesThrust() {
    auto controller = makeController();

    const double collective =
        controller.update(1.0, 1.5, 0.01);

    assert(collective < 0.5);
}

void testUpperSaturation() {
    auto controller = makeController();

    const double collective =
        controller.update(100.0, 0.0, 0.01);

    assert(collective <= 1.0);
    assert(collective >= 0.0);
}

void testLowerSaturation() {
    auto controller = makeController();

    const double collective =
        controller.update(0.0, 100.0, 0.01);

    assert(collective <= 1.0);
    assert(collective >= 0.0);
}

void testInvalidInputFailsSafe() {
    auto controller = makeController();

    const double invalid_dt =
        controller.update(1.0, 0.0, 1.0);

    /*
     * PID rejects the invalid timestep and returns
     * zero correction, therefore the altitude
     * controller falls back to hover collective.
     */
    assert(nearlyEqual(invalid_dt, 0.5));
}

void testReset() {
    auto controller = makeController();

    controller.update(1.0, 0.0, 0.01);
    controller.reset();

    const double output =
        controller.update(1.0, 1.0, 0.01);

    assert(nearlyEqual(output, 0.5));
}

}  // namespace

int main() {
    testZeroAltitudeError();
    testBelowSetpointIncreasesThrust();
    testAboveSetpointReducesThrust();
    testUpperSaturation();
    testLowerSaturation();
    testInvalidInputFailsSafe();
    testReset();

    std::cout
        << "All altitude controller tests passed.\n";

    return 0;
}
