#pragma once

namespace flight_control {

struct Quaternion {
    double w{1.0};
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

struct Vector3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

struct Quadrotor6DOFState {
    // Position in inertial/world frame [m]
    Vector3 position{};

    // Velocity in inertial/world frame [m/s]
    Vector3 velocity{};

    // Body attitude relative to inertial frame
    Quaternion attitude{};

    // Body angular velocity [rad/s]
    Vector3 angular_rate{};
};

struct Quadrotor6DOFInput {
    // Total thrust along body +Z [N]
    double total_thrust{0.0};

    // Body moments [N*m]
    Vector3 torque{};
};

struct Quadrotor6DOFParameters {
    double mass{2.0};

    double inertia_xx{0.03};
    double inertia_yy{0.03};
    double inertia_zz{0.05};

    double gravity{9.81};

    // Simple linear translational damping [N/(m/s)]
    double linear_drag_x{0.15};
    double linear_drag_y{0.15};
    double linear_drag_z{0.20};

    // Rotational damping [N*m/(rad/s)]
    double angular_damping_x{0.05};
    double angular_damping_y{0.05};
    double angular_damping_z{0.08};
};

class Quadrotor6DOF {
public:
    explicit Quadrotor6DOF(
        const Quadrotor6DOFParameters& parameters = {});

    void update(
        const Quadrotor6DOFInput& input,
        double dt);

    void reset();

    void setState(
        const Quadrotor6DOFState& state);

    const Quadrotor6DOFState&
    state() const noexcept;

    Vector3 eulerAngles() const;

private:
    static bool finite(double value) noexcept;

    static Quaternion normalize(
        const Quaternion& q);

    static Vector3 rotateBodyToInertial(
        const Quaternion& q,
        const Vector3& vector);

    static Quaternion quaternionDerivative(
        const Quaternion& q,
        const Vector3& omega);

    void validateParameters(
        const Quadrotor6DOFParameters& parameters) const;

    Quadrotor6DOFParameters parameters_;
    Quadrotor6DOFState state_;
};

}  // namespace flight_control
