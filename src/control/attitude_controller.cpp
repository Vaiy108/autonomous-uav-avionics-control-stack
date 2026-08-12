#include "attitude_controller.hpp"

#include <cmath>

namespace flight_control {

namespace {

constexpr double kPi =
    3.14159265358979323846;

constexpr double kTwoPi =
    2.0 * kPi;

}  // namespace

AttitudeController::AttitudeController(
    const AttitudeControllerConfig& config)
    : roll_controller_(config.roll),
      pitch_controller_(config.pitch),
      yaw_controller_(config.yaw) {}

BodyTorqueCommand AttitudeController::update(
    const Attitude& setpoint,
    const Attitude& measurement,
    const double dt) {

    const double roll_error =
        wrapAngle(setpoint.roll - measurement.roll);

    const double pitch_error =
        wrapAngle(setpoint.pitch - measurement.pitch);

    const double yaw_error =
        wrapAngle(setpoint.yaw - measurement.yaw);

    BodyTorqueCommand command;

    command.roll =
        roll_controller_.update(
            roll_error,
            0.0,
            dt);

    command.pitch =
        pitch_controller_.update(
            pitch_error,
            0.0,
            dt);

    command.yaw =
        yaw_controller_.update(
            yaw_error,
            0.0,
            dt);

    return command;
}

void AttitudeController::reset() {
    roll_controller_.reset();
    pitch_controller_.reset();
    yaw_controller_.reset();
}

double AttitudeController::wrapAngle(
    const double angle) {

    double wrapped =
        std::fmod(angle + kPi, kTwoPi);

    if (wrapped < 0.0) {
        wrapped += kTwoPi;
    }

    return wrapped - kPi;
}

}  // namespace flight_control
