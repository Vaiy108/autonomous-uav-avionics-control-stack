#pragma once

#include "pid_controller.hpp"

namespace flight_control {

struct Position2D {
    double x{0.0};
    double y{0.0};
};

struct PositionControllerConfig {
    PIDConfig x_pid;
    PIDConfig y_pid;

    // Maximum commanded roll/pitch angle [rad].
    double max_tilt_rad{0.35};

    // Gravity [m/s^2].
    double gravity{9.81};
};

struct PositionAttitudeSetpoint {
    double roll{0.0};
    double pitch{0.0};
};

class PositionController {
public:
    explicit PositionController(
        const PositionControllerConfig& config);

    PositionAttitudeSetpoint update(
        const Position2D& setpoint,
        const Position2D& measurement,
        double dt);

    void reset();

private:
    static double clamp(
        double value,
        double minimum,
        double maximum);

    static bool finite(double value) noexcept;

    PositionControllerConfig config_;

    PIDController x_controller_;
    PIDController y_controller_;
};

}  // namespace flight_control
