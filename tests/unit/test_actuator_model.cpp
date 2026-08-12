#include "actuator_model.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

using flight_control::ActuatorModel;
using flight_control::MotorCommands;

namespace {

bool nearlyEqual(double a, double b, double tol = 1e-9) {
    return std::abs(a - b) < tol;
}

void testEqualMotorThrust() {
    ActuatorModel model;

    MotorCommands motors;
    motors.front_left = 0.5;
    motors.front_right = 0.5;
    motors.rear_left = 0.5;
    motors.rear_right = 0.5;

    const auto forces = model.calculate(motors);

    assert(nearlyEqual(forces.total_thrust, 20.0));
    assert(nearlyEqual(forces.roll_torque, 0.0));
    assert(nearlyEqual(forces.pitch_torque, 0.0));
    assert(nearlyEqual(forces.yaw_torque, 0.0));
}

void testPositiveRollTorque() {
    ActuatorModel model;

    MotorCommands motors;
    motors.front_left = 0.6;
    motors.front_right = 0.4;
    motors.rear_left = 0.6;
    motors.rear_right = 0.4;

    const auto forces = model.calculate(motors);

    assert(forces.roll_torque > 0.0);
}

void testPositivePitchTorque() {
    ActuatorModel model;

    MotorCommands motors;
    motors.front_left = 0.4;
    motors.front_right = 0.4;
    motors.rear_left = 0.6;
    motors.rear_right = 0.6;

    const auto forces = model.calculate(motors);

    assert(forces.pitch_torque > 0.0);
}

void testPositiveYawTorque() {
    ActuatorModel model;

    MotorCommands motors;
    motors.front_left = 0.6;
    motors.front_right = 0.4;
    motors.rear_left = 0.4;
    motors.rear_right = 0.6;

    const auto forces = model.calculate(motors);

    assert(forces.yaw_torque > 0.0);
}

}  // namespace

int main() {
    testEqualMotorThrust();
    testPositiveRollTorque();
    testPositivePitchTorque();
    testPositiveYawTorque();

    std::cout << "All actuator model tests passed.\n";

    return 0;
}
