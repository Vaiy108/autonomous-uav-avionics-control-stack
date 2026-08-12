#include "quadrotor_6dof.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace flight_control {

Quadrotor6DOF::Quadrotor6DOF(
    const Quadrotor6DOFParameters& parameters)
    : parameters_(parameters) {

    validateParameters(parameters_);
}

void Quadrotor6DOF::update(
    const Quadrotor6DOFInput& input,
    const double dt) {

    if (!finite(dt) ||
        dt <= 0.0 ||
        dt > 0.1) {

        return;
    }

    if (!finite(input.total_thrust) ||
        !finite(input.torque.x) ||
        !finite(input.torque.y) ||
        !finite(input.torque.z)) {

        return;
    }

    /*
     * ----------------------------------------------------
     * Translational dynamics
     * ----------------------------------------------------
     */

    const Vector3 thrust_body{
        0.0,
        0.0,
        input.total_thrust
    };

    const Vector3 thrust_inertial =
        rotateBodyToInertial(
            state_.attitude,
            thrust_body);

    const Vector3 drag_force{
        -parameters_.linear_drag_x *
            state_.velocity.x,

        -parameters_.linear_drag_y *
            state_.velocity.y,

        -parameters_.linear_drag_z *
            state_.velocity.z
    };

    Vector3 acceleration;

    acceleration.x =
        (thrust_inertial.x +
         drag_force.x) /
        parameters_.mass;

    acceleration.y =
        (thrust_inertial.y +
         drag_force.y) /
        parameters_.mass;

    acceleration.z =
        (thrust_inertial.z +
         drag_force.z) /
        parameters_.mass -
        parameters_.gravity;

    state_.velocity.x +=
        acceleration.x * dt;

    state_.velocity.y +=
        acceleration.y * dt;

    state_.velocity.z +=
        acceleration.z * dt;

    state_.position.x +=
        state_.velocity.x * dt;

    state_.position.y +=
        state_.velocity.y * dt;

    state_.position.z +=
        state_.velocity.z * dt;

    /*
     * ----------------------------------------------------
     * Rotational rigid-body dynamics
     *
     * I * omega_dot =
     *     torque
     *     - omega x (I * omega)
     *     - damping
     * ----------------------------------------------------
     */

    const double p =
        state_.angular_rate.x;

    const double q =
        state_.angular_rate.y;

    const double r =
        state_.angular_rate.z;

    const double Ix =
        parameters_.inertia_xx;

    const double Iy =
        parameters_.inertia_yy;

    const double Iz =
        parameters_.inertia_zz;

    const double p_dot =
        (
            input.torque.x
            - (Iz - Iy) * q * r
            - parameters_.angular_damping_x * p
        ) / Ix;

    const double q_dot =
        (
            input.torque.y
            - (Ix - Iz) * p * r
            - parameters_.angular_damping_y * q
        ) / Iy;

    const double r_dot =
        (
            input.torque.z
            - (Iy - Ix) * p * q
            - parameters_.angular_damping_z * r
        ) / Iz;

    state_.angular_rate.x +=
        p_dot * dt;

    state_.angular_rate.y +=
        q_dot * dt;

    state_.angular_rate.z +=
        r_dot * dt;

    /*
     * ----------------------------------------------------
     * Quaternion attitude integration
     * ----------------------------------------------------
     */

    const Quaternion q_dot_attitude =
        quaternionDerivative(
            state_.attitude,
            state_.angular_rate);

    state_.attitude.w +=
        q_dot_attitude.w * dt;

    state_.attitude.x +=
        q_dot_attitude.x * dt;

    state_.attitude.y +=
        q_dot_attitude.y * dt;

    state_.attitude.z +=
        q_dot_attitude.z * dt;

    state_.attitude =
        normalize(state_.attitude);

    /*
     * Simple ground constraint.
     */
    if (state_.position.z < 0.0) {

        state_.position.z = 0.0;

        if (state_.velocity.z < 0.0) {
            state_.velocity.z = 0.0;
        }
    }
}

void Quadrotor6DOF::reset() {
    state_ = {};
    state_.attitude.w = 1.0;
}

void Quadrotor6DOF::setState(
    const Quadrotor6DOFState& state) {

    state_ = state;

    state_.attitude =
        normalize(state_.attitude);
}

const Quadrotor6DOFState&
Quadrotor6DOF::state() const noexcept {

    return state_;
}

Vector3 Quadrotor6DOF::eulerAngles() const {

    const Quaternion q =
        normalize(state_.attitude);

    Vector3 angles;

    // Roll
    const double sinr_cosp =
        2.0 *
        (q.w * q.x + q.y * q.z);

    const double cosr_cosp =
        1.0 -
        2.0 *
        (q.x * q.x + q.y * q.y);

    angles.x =
        std::atan2(
            sinr_cosp,
            cosr_cosp);

    // Pitch
    const double sinp =
        2.0 *
        (q.w * q.y - q.z * q.x);

    if (std::abs(sinp) >= 1.0) {

        angles.y =
            std::copysign(
                3.14159265358979323846 / 2.0,
                sinp);

    } else {

        angles.y =
            std::asin(sinp);
    }

    // Yaw
    const double siny_cosp =
        2.0 *
        (q.w * q.z + q.x * q.y);

    const double cosy_cosp =
        1.0 -
        2.0 *
        (q.y * q.y + q.z * q.z);

    angles.z =
        std::atan2(
            siny_cosp,
            cosy_cosp);

    return angles;
}

Quaternion Quadrotor6DOF::normalize(
    const Quaternion& q) {

    const double norm =
        std::sqrt(
            q.w * q.w +
            q.x * q.x +
            q.y * q.y +
            q.z * q.z);

    if (!finite(norm) ||
        norm < 1e-12) {

        return {};
    }

    return {
        q.w / norm,
        q.x / norm,
        q.y / norm,
        q.z / norm
    };
}

Vector3 Quadrotor6DOF::rotateBodyToInertial(
    const Quaternion& q_in,
    const Vector3& v) {

    const Quaternion q =
        normalize(q_in);

    /*
     * Rotation matrix corresponding to quaternion.
     */

    Vector3 result;

    result.x =
        (1.0 - 2.0 * (q.y*q.y + q.z*q.z)) * v.x +
        (2.0 * (q.x*q.y - q.w*q.z)) * v.y +
        (2.0 * (q.x*q.z + q.w*q.y)) * v.z;

    result.y =
        (2.0 * (q.x*q.y + q.w*q.z)) * v.x +
        (1.0 - 2.0 * (q.x*q.x + q.z*q.z)) * v.y +
        (2.0 * (q.y*q.z - q.w*q.x)) * v.z;

    result.z =
        (2.0 * (q.x*q.z - q.w*q.y)) * v.x +
        (2.0 * (q.y*q.z + q.w*q.x)) * v.y +
        (1.0 - 2.0 * (q.x*q.x + q.y*q.y)) * v.z;

    return result;
}

Quaternion Quadrotor6DOF::quaternionDerivative(
    const Quaternion& q,
    const Vector3& omega) {

    Quaternion derivative;

    derivative.w =
        -0.5 *
        (
            q.x * omega.x +
            q.y * omega.y +
            q.z * omega.z
        );

    derivative.x =
        0.5 *
        (
            q.w * omega.x +
            q.y * omega.z -
            q.z * omega.y
        );

    derivative.y =
        0.5 *
        (
            q.w * omega.y +
            q.z * omega.x -
            q.x * omega.z
        );

    derivative.z =
        0.5 *
        (
            q.w * omega.z +
            q.x * omega.y -
            q.y * omega.x
        );

    return derivative;
}

bool Quadrotor6DOF::finite(
    const double value) noexcept {

    return std::isfinite(value);
}

void Quadrotor6DOF::validateParameters(
    const Quadrotor6DOFParameters& p) const {

    if (!finite(p.mass) ||
        !finite(p.inertia_xx) ||
        !finite(p.inertia_yy) ||
        !finite(p.inertia_zz) ||
        !finite(p.gravity) ||
        p.mass <= 0.0 ||
        p.inertia_xx <= 0.0 ||
        p.inertia_yy <= 0.0 ||
        p.inertia_zz <= 0.0 ||
        p.gravity <= 0.0) {

        throw std::invalid_argument(
            "Invalid 6-DOF quadrotor parameters");
    }

    if (p.linear_drag_x < 0.0 ||
        p.linear_drag_y < 0.0 ||
        p.linear_drag_z < 0.0 ||
        p.angular_damping_x < 0.0 ||
        p.angular_damping_y < 0.0 ||
        p.angular_damping_z < 0.0) {

        throw std::invalid_argument(
            "Damping coefficients cannot be negative");
    }
}

}  // namespace flight_control
