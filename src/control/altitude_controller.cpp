#include "altitude_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace flight_control {

AltitudeController::AltitudeController(
    const AltitudeControllerConfig& config)
    : config_(config),
      altitude_controller_(config.altitude_pid) {

    if (!std::isfinite(config_.hover_collective) ||
        !std::isfinite(config_.collective_min) ||
        !std::isfinite(config_.collective_max) ||
        config_.collective_min >= config_.collective_max ||
        config_.hover_collective < config_.collective_min ||
        config_.hover_collective > config_.collective_max) {

        throw std::invalid_argument(
            "Altitude controller configuration is invalid");
    }
}

double AltitudeController::update(
    const double altitude_setpoint,
    const double altitude_measurement,
    const double dt) {

    if (!std::isfinite(altitude_setpoint) ||
        !std::isfinite(altitude_measurement) ||
        !std::isfinite(dt)) {

        return config_.hover_collective;
    }

    const double correction =
        altitude_controller_.update(
            altitude_setpoint,
            altitude_measurement,
            dt);

    return clamp(
        config_.hover_collective + correction,
        config_.collective_min,
        config_.collective_max);
}

void AltitudeController::reset() {
    altitude_controller_.reset();
}

double AltitudeController::clamp(
    const double value,
    const double minimum,
    const double maximum) {

    return std::clamp(value, minimum, maximum);
}

}  // namespace flight_control
