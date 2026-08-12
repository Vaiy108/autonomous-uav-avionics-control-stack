#include "quadrotor_6dof.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

using flight_control::Quadrotor6DOF;
using flight_control::Quadrotor6DOFInput;
using flight_control::Quadrotor6DOFParameters;
using flight_control::Quadrotor6DOFState;
using flight_control::Quaternion;

namespace {

bool nearlyEqual(double a, double b, double tol = 1e-6) {
    return std::abs(a - b) < tol;
}

double quaternionNorm(const Quaternion& q) {
    return std::sqrt(
        q.w * q.w +
        q.x * q.x +
        q.y * q.y +
        q.z * q.z
    );
}

void testInitialState() {
    Quadrotor6DOF dynamics;

    const auto& state = dynamics.state();

    assert(nearlyEqual(state.position.x, 0.0));
    assert(nearlyEqual(state.position.y, 0.0));
    assert(nearlyEqual(state.position.z, 0.0));

    assert(nearlyEqual(state.velocity.x, 0.0));
    assert(nearlyEqual(state.velocity.y, 0.0));
    assert(nearlyEqual(state.velocity.z, 0.0));

    assert(nearlyEqual(state.attitude.w, 1.0));
}

void testHover() {
    Quadrotor6DOFParameters params;
    Quadrotor6DOF dynamics(params);

    Quadrotor6DOFState initial;
    initial.position.z = 1.0;
    dynamics.setState(initial);

    Quadrotor6DOFInput input;
    input.total_thrust =
        params.mass * params.gravity;

    for (int i = 0; i < 500; ++i) {
        dynamics.update(input, 0.01);
    }

    const auto& state = dynamics.state();

    assert(std::abs(state.position.z - 1.0) < 1e-3);
    assert(std::abs(state.velocity.z) < 1e-3);
}

void testPositiveRollTorque() {
    Quadrotor6DOF dynamics;

    Quadrotor6DOFInput input;
    input.torque.x = 0.1;

    for (int i = 0; i < 20; ++i) {
        dynamics.update(input, 0.01);
    }

    const auto& state = dynamics.state();
    const auto angles = dynamics.eulerAngles();

    assert(state.angular_rate.x > 0.0);
    assert(angles.x > 0.0);
}

void testQuaternionNormalization() {
    Quadrotor6DOF dynamics;

    Quadrotor6DOFInput input;
    input.torque.x = 0.10;
    input.torque.y = 0.08;
    input.torque.z = 0.05;

    for (int i = 0; i < 1000; ++i) {
        dynamics.update(input, 0.005);
    }

    const double norm =
        quaternionNorm(
            dynamics.state().attitude);

    assert(std::abs(norm - 1.0) < 1e-9);
}

void testTiltProducesHorizontalMotion() {
    Quadrotor6DOFParameters params;
    Quadrotor6DOF dynamics(params);

    constexpr double roll =
        15.0 * 3.14159265358979323846 / 180.0;

    Quadrotor6DOFState initial;
    initial.position.z = 1.0;

    initial.attitude.w =
        std::cos(roll / 2.0);

    initial.attitude.x =
        std::sin(roll / 2.0);

    dynamics.setState(initial);

    Quadrotor6DOFInput input;

    /*
     * Slightly compensate total thrust for the
     * vertical loss due to the initial roll angle.
     */
    input.total_thrust =
        params.mass *
        params.gravity /
        std::cos(roll);

    for (int i = 0; i < 100; ++i) {
        dynamics.update(input, 0.01);
    }

    const auto& state = dynamics.state();

    /*
     * A rolled thrust vector must create
     * non-zero horizontal motion.
     *
     * Depending on the coordinate convention,
     * the sign may be positive or negative.
     */
    assert(std::abs(state.position.y) > 1e-3);
    assert(std::abs(state.velocity.y) > 1e-3);
}

void testGroundConstraint() {
    Quadrotor6DOF dynamics;

    Quadrotor6DOFState initial;
    initial.position.z = 0.1;
    dynamics.setState(initial);

    Quadrotor6DOFInput input;
    input.total_thrust = 0.0;

    for (int i = 0; i < 500; ++i) {
        dynamics.update(input, 0.01);
    }

    assert(nearlyEqual(
        dynamics.state().position.z,
        0.0));

    assert(dynamics.state().velocity.z >= 0.0);
}

void testInvalidTimestepIgnored() {
    Quadrotor6DOF dynamics;

    Quadrotor6DOFInput input;
    input.torque.x = 1.0;

    dynamics.update(input, 1.0);

    const auto angles =
        dynamics.eulerAngles();

    assert(nearlyEqual(angles.x, 0.0));
    assert(nearlyEqual(
        dynamics.state().angular_rate.x,
        0.0));
}

}  // namespace

int main() {
    testInitialState();
    testHover();
    testPositiveRollTorque();
    testQuaternionNormalization();
    testTiltProducesHorizontalMotion();
    testGroundConstraint();
    testInvalidTimestepIgnored();

    std::cout
        << "All 6-DOF quadrotor dynamics tests passed.\n";

    return 0;
}
