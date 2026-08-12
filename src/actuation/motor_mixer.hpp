#pragma once

namespace flight_control {

struct MotorCommands {
    double front_left{0.0};
    double front_right{0.0};
    double rear_left{0.0};
    double rear_right{0.0};

    bool saturated{false};
};

struct MixerInput {
    double collective{0.0};

    double roll{0.0};
    double pitch{0.0};
    double yaw{0.0};
};

struct MotorMixerConfig {
    double output_min{0.0};
    double output_max{1.0};
};

class MotorMixer {
public:
    explicit MotorMixer(
        const MotorMixerConfig& config = {});

    MotorCommands mix(const MixerInput& input) const;

private:
    static double clamp(
        double value,
        double minimum,
        double maximum);

    static bool finite(double value) noexcept;

    MotorMixerConfig config_;
};

}  // namespace flight_control
