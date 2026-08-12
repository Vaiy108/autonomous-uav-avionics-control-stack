#include "quadrotor_dynamics.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

using flight_control::QuadrotorDynamics;
using flight_control::QuadrotorInput;
using flight_control::QuadrotorParameters;
using flight_control::QuadrotorState;

namespace {

bool nearlyEqual(double a, double b, double tol = 1e-6) {
    return std::abs(a - b) < tol;
}

void testInitialState() {
    QuadrotorDynamics dynamics;

    const auto& state = dynamics.state();

    assert(nearlyEqual(state.roll, 0.0));
    assert(nearlyEqual(state.pitch, 0.0));
    assert(nearlyEqual(state.yaw, 0.0));
    assert(nearlyEqual(state.altitude, 0.0));
}

void testPositiveRollTorque() {
    QuadrotorDynamics dynamics;

    QuadrotorInput input;
    input.roll_torque = 0.1;

    dynamics.update(input, 0.01);

    const auto& state = dynamics.state();

    assert(state.roll_rate > 0.0);
    assert(state.roll > 0.0);
}

void testPositivePitchTorque() {
    QuadrotorDynamics dynamics;

    QuadrotorInput input;
    input.pitch_torque = 0.1;

    dynamics.update(input, 0.01);

    const auto& state = dynamics.state();

    assert(state.pitch_rate > 0.0);
    assert(state.pitch > 0.0);
}

void testPositiveYawTorque() {
    QuadrotorDynamics dynamics;

    QuadrotorInput input;
    input.yaw_torque = 0.1;

    dynamics.update(input, 0.01);

    const auto& state = dynamics.state();

    assert(state.yaw_rate > 0.0);
    assert(state.yaw > 0.0);
}

void testApproximateHover() {
    QuadrotorParameters params;

    QuadrotorDynamics dynamics(params);

    QuadrotorState initial;
    initial.altitude = 1.0;

    dynamics.setState(initial);

    QuadrotorInput input;

    input.collective_thrust =
        (params.mass * params.gravity) /
        params.max_total_thrust;

    for (int i = 0; i < 100; ++i) {
        dynamics.update(input, 0.01);
    }

    const auto& state = dynamics.state();

    assert(std::abs(state.altitude - 1.0) < 1e-3);
    assert(std::abs(state.vertical_velocity) < 1e-3);
}

void testBelowHoverDescends() {
    QuadrotorParameters params;

    QuadrotorDynamics dynamics(params);

    QuadrotorState initial;
    initial.altitude = 1.0;

    dynamics.setState(initial);

    QuadrotorInput input;
    input.collective_thrust = 0.4;

    for (int i = 0; i < 50; ++i) {
        dynamics.update(input, 0.01);
    }

    assert(dynamics.state().altitude < 1.0);
}

void testReset() {
    QuadrotorDynamics dynamics;

    QuadrotorInput input;
    input.roll_torque = 0.1;

    dynamics.update(input, 0.01);

    assert(dynamics.state().roll != 0.0);

    dynamics.reset();

    assert(nearlyEqual(dynamics.state().roll, 0.0));
    assert(nearlyEqual(dynamics.state().roll_rate, 0.0));
    assert(nearlyEqual(dynamics.state().altitude, 0.0));
}

void testInvalidTimestepIgnored() {
    QuadrotorDynamics dynamics;

    QuadrotorInput input;
    input.roll_torque = 1.0;

    dynamics.update(input, 1.0);

    assert(nearlyEqual(dynamics.state().roll, 0.0));
    assert(nearlyEqual(dynamics.state().roll_rate, 0.0));
}

}  // namespace

int main() {
    testInitialState();
    testPositiveRollTorque();
    testPositivePitchTorque();
    testPositiveYawTorque();
    testApproximateHover();
    testBelowHoverDescends();
    testReset();
    testInvalidTimestepIgnored();

    std::cout
        << "All quadrotor dynamics tests passed.\n";

    return 0;
}
