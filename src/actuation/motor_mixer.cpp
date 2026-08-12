#include "motor_mixer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace flight_control {

MotorMixer::MotorMixer(
    const MotorMixerConfig& config)
    : config_(config) {

    if (!finite(config_.output_min) ||
        !finite(config_.output_max) ||
        config_.output_min >= config_.output_max) {

        throw std::invalid_argument(
            "Motor mixer output limits are invalid");
    }
}

MotorCommands MotorMixer::mix(
    const MixerInput& input) const {

    MotorCommands output;

    if (!finite(input.collective) ||
        !finite(input.roll) ||
        !finite(input.pitch) ||
        !finite(input.yaw)) {

        return output;
    }

    /*
     * X-configuration quadrotor mixer.
     *
     * Motor layout viewed from above:
     *
     *          FRONT
     *
     *       FL       FR
     *        \       /
     *         \     /
     *          \   /
     *           UAV
     *          /   \
     *         /     \
     *        /       \
     *       RL       RR
     *
     * Commands are normalized.
     *
     * Sign convention:
     *   +roll  -> increase left / decrease right
     *   +pitch -> increase rear / decrease front
     *   +yaw   -> differential rotor torque
     */

    const double fl =
        input.collective +
        input.roll -
        input.pitch +
        input.yaw;

    const double fr =
        input.collective -
        input.roll -
        input.pitch -
        input.yaw;

    const double rl =
        input.collective +
        input.roll +
        input.pitch -
        input.yaw;

    const double rr =
        input.collective -
        input.roll +
        input.pitch +
        input.yaw;

    output.saturated =
        fl < config_.output_min ||
        fl > config_.output_max ||
        fr < config_.output_min ||
        fr > config_.output_max ||
        rl < config_.output_min ||
        rl > config_.output_max ||
        rr < config_.output_min ||
        rr > config_.output_max;

    output.front_left =
        clamp(fl, config_.output_min, config_.output_max);

    output.front_right =
        clamp(fr, config_.output_min, config_.output_max);

    output.rear_left =
        clamp(rl, config_.output_min, config_.output_max);

    output.rear_right =
        clamp(rr, config_.output_min, config_.output_max);

    return output;
}

double MotorMixer::clamp(
    const double value,
    const double minimum,
    const double maximum) {

    return std::clamp(value, minimum, maximum);
}

bool MotorMixer::finite(
    const double value) noexcept {

    return std::isfinite(value);
}

}  // namespace flight_control
