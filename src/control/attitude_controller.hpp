#pragma once

#include "pid_controller.hpp"

namespace flight_control {

struct Attitude {
    double roll{0.0};
    double pitch{0.0};
    double yaw{0.0};
};

struct BodyTorqueCommand {
    double roll{0.0};
    double pitch{0.0};
    double yaw{0.0};
};

struct AttitudeControllerConfig {
    PIDConfig roll;
    PIDConfig pitch;
    PIDConfig yaw;
};

class AttitudeController {
public:
    explicit AttitudeController(
        const AttitudeControllerConfig& config);

    BodyTorqueCommand update(
        const Attitude& setpoint,
        const Attitude& measurement,
        double dt);

    void reset();

private:
    static double wrapAngle(double angle);

    PIDController roll_controller_;
    PIDController pitch_controller_;
    PIDController yaw_controller_;
};

}  // namespace flight_control
