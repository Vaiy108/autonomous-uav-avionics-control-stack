#include "pid_controller.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

using flight_control::PIDConfig;
using flight_control::PIDController;

namespace {

bool nearlyEqual(double a, double b, double tol = 1e-9) {
    return std::abs(a - b) < tol;
}

void testProportionalResponse() {
    PIDConfig cfg;
    cfg.kp = 2.0;
    cfg.ki = 0.0;
    cfg.kd = 0.0;
    cfg.output_min = -10.0;
    cfg.output_max = 10.0;

    PIDController pid(cfg);

    const double output = pid.update(1.0, 0.5, 0.01);

    assert(nearlyEqual(output, 1.0));
}

void testOutputSaturation() {
    PIDConfig cfg;
    cfg.kp = 10.0;
    cfg.output_min = -1.0;
    cfg.output_max = 1.0;

    PIDController pid(cfg);

    const double high = pid.update(10.0, 0.0, 0.01);

    pid.reset();

    const double low = pid.update(-10.0, 0.0, 0.01);

    assert(nearlyEqual(high, 1.0));
    assert(nearlyEqual(low, -1.0));
}

void testReset() {
    PIDConfig cfg;
    cfg.kp = 1.0;
    cfg.ki = 1.0;
    cfg.kd = 0.0;

    PIDController pid(cfg);

    pid.update(1.0, 0.0, 0.01);

    assert(pid.initialized());

    pid.reset();

    assert(!pid.initialized());
    assert(nearlyEqual(pid.integralState(), 0.0));
    assert(nearlyEqual(pid.previousError(), 0.0));
    assert(nearlyEqual(pid.derivativeState(), 0.0));
}

void testInvalidTimestep() {
    PIDConfig cfg;
    cfg.kp = 1.0;

    PIDController pid(cfg);

    const double too_small =
        pid.update(1.0, 0.0, 0.0);

    const double too_large =
        pid.update(1.0, 0.0, 1.0);

    assert(nearlyEqual(too_small, 0.0));
    assert(nearlyEqual(too_large, 0.0));
}

void testAntiWindup() {
    PIDConfig cfg;
    cfg.kp = 10.0;
    cfg.ki = 5.0;
    cfg.kd = 0.0;

    cfg.output_min = -1.0;
    cfg.output_max = 1.0;

    cfg.integral_min = -10.0;
    cfg.integral_max = 10.0;

    PIDController pid(cfg);

    for (int i = 0; i < 100; ++i) {
        pid.update(10.0, 0.0, 0.01);
    }

    // Controller is saturated high, so the integrator
    // should not continue winding up.
    assert(nearlyEqual(pid.integralState(), 0.0));
}

void testConfigurationValidation() {
    PIDConfig cfg;
    cfg.output_min = 1.0;
    cfg.output_max = -1.0;

    bool exception_thrown = false;

    try {
        PIDController pid(cfg);
    } catch (const std::invalid_argument&) {
        exception_thrown = true;
    }

    assert(exception_thrown);
}

}  // namespace

int main() {
    testProportionalResponse();
    testOutputSaturation();
    testReset();
    testInvalidTimestep();
    testAntiWindup();
    testConfigurationValidation();

    std::cout << "All PID controller tests passed.\n";

    return 0;
}
