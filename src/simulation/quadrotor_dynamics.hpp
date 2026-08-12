#pragma once

namespace flight_control {

struct QuadrotorState {
    double roll{0.0};
    double pitch{0.0};
    double yaw{0.0};

    double roll_rate{0.0};
    double pitch_rate{0.0};
    double yaw_rate{0.0};

    double altitude{0.0};
    double vertical_velocity{0.0};
};

struct QuadrotorInput {
    double roll_torque{0.0};
    double pitch_torque{0.0};
    double yaw_torque{0.0};

    double collective_thrust{0.0};
};

struct QuadrotorParameters {
    double mass{2.0};

    double inertia_xx{0.03};
    double inertia_yy{0.03};
    double inertia_zz{0.05};

    double gravity{9.81};

    double angular_damping_roll{0.05};
    double angular_damping_pitch{0.05};
    double angular_damping_yaw{0.08};

    double vertical_damping{0.15};

    double max_total_thrust{40.0};
};

class QuadrotorDynamics {
public:
    explicit QuadrotorDynamics(
        const QuadrotorParameters& params = {});

    void reset();

    void setState(const QuadrotorState& state);

    const QuadrotorState& state() const noexcept;

    void update(
        const QuadrotorInput& input,
        double dt);

private:
    static bool finite(double value) noexcept;

    void validateParameters(
        const QuadrotorParameters& params) const;

    QuadrotorParameters params_;
    QuadrotorState state_;
};

}  // namespace flight_control
