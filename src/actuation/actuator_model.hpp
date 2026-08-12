#pragma once

#include "motor_mixer.hpp"

namespace flight_control {

struct ActuatorForces {
    double total_thrust{0.0};

    double roll_torque{0.0};
    double pitch_torque{0.0};
    double yaw_torque{0.0};
};

struct ActuatorModelConfig {
    // Distance from vehicle center to motor [m].
    double arm_length{0.25};

    // Maximum thrust produced by one motor [N].
    double max_motor_thrust{10.0};

    // Approximate reaction-torque coefficient [N*m / N].
    double yaw_torque_coefficient{0.02};
};

class ActuatorModel {
public:
    explicit ActuatorModel(
        const ActuatorModelConfig& config = {});

    ActuatorForces calculate(
        const MotorCommands& motors) const;

private:
    static bool finite(double value) noexcept;

    ActuatorModelConfig config_;
};

}  // namespace flight_control
