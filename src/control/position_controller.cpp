#include "position_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace flight_control {

PositionController::PositionController(
    const PositionControllerConfig& config)
    : config_(config),
      x_controller_(config.x_pid),
      y_controller_(config.y_pid) {

    if (!finite(config_.max_tilt_rad) ||
        !finite(config_.gravity) ||
        config_.max_tilt_rad <= 0.0 ||
        config_.gravity <= 0.0) {

        throw std::invalid_argument(
            "Position controller configuration is invalid");
    }
}

PositionAttitudeSetpoint PositionController::update(
    const Position2D& setpoint,
    const Position2D& measurement,
    const double dt) {

    PositionAttitudeSetpoint output;

    if (!finite(setpoint.x) ||
        !finite(setpoint.y) ||
        !finite(measurement.x) ||
        !finite(measurement.y) ||
        !finite(dt)) {

        return output;
    }

    /*
     * Outer-loop PID outputs are interpreted as
     * desired horizontal accelerations [m/s^2].
     */
    const double accel_x =
        x_controller_.update(
            setpoint.x,
            measurement.x,
            dt);

    const double accel_y =
        y_controller_.update(
            setpoint.y,
            measurement.y,
            dt);

    /*
     * Small-angle approximation:
     *
     * ax ~=  g * pitch
     * ay ~= -g * roll
     *
     * Sign convention follows the current
     * 6-DOF model and thrust-vector orientation.
     */
    output.pitch =
        clamp(
            accel_x / config_.gravity,
            -config_.max_tilt_rad,
            config_.max_tilt_rad);

    output.roll =
        clamp(
            -accel_y / config_.gravity,
            -config_.max_tilt_rad,
            config_.max_tilt_rad);

    return output;
}

void PositionController::reset() {
    x_controller_.reset();
    y_controller_.reset();
}

double PositionController::clamp(
    const double value,
    const double minimum,
    const double maximum) {

    return std::clamp(value, minimum, maximum);
}

bool PositionController::finite(
    const double value) noexcept {

    return std::isfinite(value);
}

}  // namespace flight_control
