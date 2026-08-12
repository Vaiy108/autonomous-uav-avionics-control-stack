#include "actuator_model.hpp"

#include <cmath>
#include <stdexcept>

namespace flight_control {

ActuatorModel::ActuatorModel(
    const ActuatorModelConfig& config)
    : config_(config) {

    if (!finite(config_.arm_length) ||
        !finite(config_.max_motor_thrust) ||
        !finite(config_.yaw_torque_coefficient) ||
        config_.arm_length <= 0.0 ||
        config_.max_motor_thrust <= 0.0 ||
        config_.yaw_torque_coefficient < 0.0) {

        throw std::invalid_argument(
            "Actuator model configuration is invalid");
    }
}

ActuatorForces ActuatorModel::calculate(
    const MotorCommands& motors) const {

    ActuatorForces output;

    if (!finite(motors.front_left) ||
        !finite(motors.front_right) ||
        !finite(motors.rear_left) ||
        !finite(motors.rear_right)) {

        return output;
    }

    const double fl =
        motors.front_left * config_.max_motor_thrust;

    const double fr =
        motors.front_right * config_.max_motor_thrust;

    const double rl =
        motors.rear_left * config_.max_motor_thrust;

    const double rr =
        motors.rear_right * config_.max_motor_thrust;

    output.total_thrust =
        fl + fr + rl + rr;

    /*
     * Sign convention matches MotorMixer:
     *
     * +roll  -> left motors produce more thrust
     * +pitch -> rear motors produce more thrust
     */

    output.roll_torque =
        config_.arm_length *
        ((fl + rl) - (fr + rr));

    output.pitch_torque =
        config_.arm_length *
        ((rl + rr) - (fl + fr));

    /*
     * Assume FL/RR rotate in one direction
     * and FR/RL in the opposite direction.
     */
    output.yaw_torque =
        config_.yaw_torque_coefficient *
        ((fl + rr) - (fr + rl));

    return output;
}

bool ActuatorModel::finite(
    const double value) noexcept {

    return std::isfinite(value);
}

}  // namespace flight_control
