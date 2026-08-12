#include "motor_mixer.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

using flight_control::MixerInput;
using flight_control::MotorMixer;
using flight_control::MotorMixerConfig;

namespace {

bool nearlyEqual(double a, double b, double tol = 1e-9) {
    return std::abs(a - b) < tol;
}

void testCollectiveOnly() {
    MotorMixer mixer;

    MixerInput input;
    input.collective = 0.5;

    const auto output = mixer.mix(input);

    assert(nearlyEqual(output.front_left, 0.5));
    assert(nearlyEqual(output.front_right, 0.5));
    assert(nearlyEqual(output.rear_left, 0.5));
    assert(nearlyEqual(output.rear_right, 0.5));
    assert(!output.saturated);
}

void testPositiveRoll() {
    MotorMixer mixer;

    MixerInput input;
    input.collective = 0.5;
    input.roll = 0.1;

    const auto output = mixer.mix(input);

    assert(nearlyEqual(output.front_left, 0.6));
    assert(nearlyEqual(output.front_right, 0.4));
    assert(nearlyEqual(output.rear_left, 0.6));
    assert(nearlyEqual(output.rear_right, 0.4));
    assert(!output.saturated);
}

void testPositivePitch() {
    MotorMixer mixer;

    MixerInput input;
    input.collective = 0.5;
    input.pitch = 0.1;

    const auto output = mixer.mix(input);

    assert(nearlyEqual(output.front_left, 0.4));
    assert(nearlyEqual(output.front_right, 0.4));
    assert(nearlyEqual(output.rear_left, 0.6));
    assert(nearlyEqual(output.rear_right, 0.6));
}

void testPositiveYaw() {
    MotorMixer mixer;

    MixerInput input;
    input.collective = 0.5;
    input.yaw = 0.1;

    const auto output = mixer.mix(input);

    assert(nearlyEqual(output.front_left, 0.6));
    assert(nearlyEqual(output.front_right, 0.4));
    assert(nearlyEqual(output.rear_left, 0.4));
    assert(nearlyEqual(output.rear_right, 0.6));
}

void testSaturation() {
    MotorMixer mixer;

    MixerInput input;
    input.collective = 0.95;
    input.roll = 0.2;

    const auto output = mixer.mix(input);

    assert(output.saturated);

    assert(output.front_left <= 1.0);
    assert(output.front_right <= 1.0);
    assert(output.rear_left <= 1.0);
    assert(output.rear_right <= 1.0);

    assert(output.front_left >= 0.0);
    assert(output.front_right >= 0.0);
    assert(output.rear_left >= 0.0);
    assert(output.rear_right >= 0.0);
}

void testInvalidInputFailsSafe() {
    MotorMixer mixer;

    MixerInput input;
    input.collective =
        std::numeric_limits<double>::quiet_NaN();

    const auto output = mixer.mix(input);

    assert(nearlyEqual(output.front_left, 0.0));
    assert(nearlyEqual(output.front_right, 0.0));
    assert(nearlyEqual(output.rear_left, 0.0));
    assert(nearlyEqual(output.rear_right, 0.0));
}

void testCustomOutputLimits() {
    MotorMixerConfig cfg;
    cfg.output_min = 0.1;
    cfg.output_max = 0.8;

    MotorMixer mixer(cfg);

    MixerInput input;
    input.collective = 1.0;

    const auto output = mixer.mix(input);

    assert(output.saturated);

    assert(nearlyEqual(output.front_left, 0.8));
    assert(nearlyEqual(output.front_right, 0.8));
    assert(nearlyEqual(output.rear_left, 0.8));
    assert(nearlyEqual(output.rear_right, 0.8));
}

}  // namespace

int main() {
    testCollectiveOnly();
    testPositiveRoll();
    testPositivePitch();
    testPositiveYaw();
    testSaturation();
    testInvalidInputFailsSafe();
    testCustomOutputLimits();

    std::cout << "All motor mixer tests passed.\n";

    return 0;
}
