#include "quadrotor_dynamics.hpp"

#include <cmath>
#include <stdexcept>

namespace flight_control {

QuadrotorDynamics::QuadrotorDynamics(
    const QuadrotorParameters& params)
    : params_(params) {

    validateParameters(params_);
}

void QuadrotorDynamics::reset() {
    state_ = {};
}

void QuadrotorDynamics::setState(
    const QuadrotorState& state) {

    state_ = state;
}

const QuadrotorState&
QuadrotorDynamics::state() const noexcept {
    return state_;
}

void QuadrotorDynamics::update(
    const QuadrotorInput& input,
    const double dt) {

    if (!finite(dt) || dt <= 0.0 || dt > 0.1) {
        return;
    }

    if (!finite(input.roll_torque) ||
        !finite(input.pitch_torque) ||
        !finite(input.yaw_torque) ||
        !finite(input.collective_thrust)) {

        return;
    }

    /*
     * Simplified rigid-body rotational dynamics:
     *
     * I * angular_acceleration =
     *      applied_torque - damping * angular_rate
     */

    const double roll_accel =
        (input.roll_torque -
         params_.angular_damping_roll *
             state_.roll_rate) /
        params_.inertia_xx;

    const double pitch_accel =
        (input.pitch_torque -
         params_.angular_damping_pitch *
             state_.pitch_rate) /
        params_.inertia_yy;

    const double yaw_accel =
        (input.yaw_torque -
         params_.angular_damping_yaw *
             state_.yaw_rate) /
        params_.inertia_zz;

    state_.roll_rate += roll_accel * dt;
    state_.pitch_rate += pitch_accel * dt;
    state_.yaw_rate += yaw_accel * dt;

    state_.roll += state_.roll_rate * dt;
    state_.pitch += state_.pitch_rate * dt;
    state_.yaw += state_.yaw_rate * dt;

    /*
     * Vertical dynamics.
     *
     * collective_thrust is normalized [0,1].
     * Convert it into physical thrust [N].
     */

    const double total_thrust =
        input.collective_thrust *
        params_.max_total_thrust;

    /*
     * Only the vertical component contributes
     * to altitude in this simplified model.
     */

    const double vertical_thrust =
        total_thrust *
        std::cos(state_.roll) *
        std::cos(state_.pitch);

    const double vertical_accel =
        (vertical_thrust / params_.mass) -
        params_.gravity -
        params_.vertical_damping *
            state_.vertical_velocity;

    state_.vertical_velocity +=
        vertical_accel * dt;

    state_.altitude +=
        state_.vertical_velocity * dt;

    /*
     * Simple ground constraint.
     */
    if (state_.altitude < 0.0) {
        state_.altitude = 0.0;

        if (state_.vertical_velocity < 0.0) {
            state_.vertical_velocity = 0.0;
        }
    }
}

bool QuadrotorDynamics::finite(
    const double value) noexcept {

    return std::isfinite(value);
}

void QuadrotorDynamics::validateParameters(
    const QuadrotorParameters& params) const {

    if (params.mass <= 0.0 ||
        params.inertia_xx <= 0.0 ||
        params.inertia_yy <= 0.0 ||
        params.inertia_zz <= 0.0 ||
        params.gravity <= 0.0 ||
        params.max_total_thrust <= 0.0) {

        throw std::invalid_argument(
            "Quadrotor parameters must be positive");
    }

    if (!finite(params.mass) ||
        !finite(params.inertia_xx) ||
        !finite(params.inertia_yy) ||
        !finite(params.inertia_zz) ||
        !finite(params.gravity) ||
        !finite(params.max_total_thrust)) {

        throw std::invalid_argument(
            "Quadrotor parameters must be finite");
    }
}

}  // namespace flight_control
