#pragma once

#include "pid_controller.hpp"

namespace flight_control {

struct AltitudeControllerConfig {
    PIDConfig altitude_pid;

    // Base collective required approximately for hover.
    double hover_collective{0.5};

    double collective_min{0.0};
    double collective_max{1.0};
};

class AltitudeController {
public:
    explicit AltitudeController(
        const AltitudeControllerConfig& config);

    double update(
        double altitude_setpoint,
        double altitude_measurement,
        double dt);

    void reset();

private:
    static double clamp(
        double value,
        double minimum,
        double maximum);

    AltitudeControllerConfig config_;
    PIDController altitude_controller_;
};

}  // namespace flight_control
