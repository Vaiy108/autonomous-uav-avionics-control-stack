#include "position_controller.hpp"
#include "attitude_controller.hpp"
#include "altitude_controller.hpp"
#include "motor_mixer.hpp"
#include "actuator_model.hpp"
#include "quadrotor_6dof.hpp"

#include <cmath>
#include <fstream>
#include <iostream>

using namespace flight_control;

int main() {
    constexpr double dt = 0.01;
    constexpr double sim_time = 15.0;
    constexpr double max_total_thrust = 40.0;

    // -------------------------------------------------
    // Vehicle
    // -------------------------------------------------
    Quadrotor6DOFParameters params;

    params.mass = 2.0;
    params.inertia_xx = 0.03;
    params.inertia_yy = 0.03;
    params.inertia_zz = 0.05;
    params.gravity = 9.81;

    params.linear_drag_x = 0.15;
    params.linear_drag_y = 0.15;
    params.linear_drag_z = 0.20;

    params.angular_damping_x = 0.05;
    params.angular_damping_y = 0.05;
    params.angular_damping_z = 0.08;

    Quadrotor6DOF dynamics(params);

    Quadrotor6DOFState initial_state;
    initial_state.position.z = 1.0;

    dynamics.setState(initial_state);

    // -------------------------------------------------
    // Outer-loop XY position controller
    // -------------------------------------------------
    PositionControllerConfig position_cfg;

    position_cfg.gravity = params.gravity;

    // ~15 degree maximum commanded tilt
    position_cfg.max_tilt_rad =
        15.0 * M_PI / 180.0;

    position_cfg.x_pid.kp = 0.55;
    position_cfg.x_pid.ki = 0.02;
    position_cfg.x_pid.kd = 0.80;

    position_cfg.x_pid.output_min = -3.0;
    position_cfg.x_pid.output_max = 3.0;
    position_cfg.x_pid.integral_min = -2.0;
    position_cfg.x_pid.integral_max = 2.0;

    position_cfg.y_pid =
        position_cfg.x_pid;

    PositionController position_controller(
        position_cfg);

    // -------------------------------------------------
    // Inner-loop attitude controller
    // -------------------------------------------------
    AttitudeControllerConfig attitude_cfg;

    attitude_cfg.roll.kp = 0.8;
    attitude_cfg.roll.ki = 0.10;
    attitude_cfg.roll.kd = 0.08;

    attitude_cfg.roll.output_min = -0.20;
    attitude_cfg.roll.output_max = 0.20;

    attitude_cfg.pitch =
        attitude_cfg.roll;

    attitude_cfg.yaw.kp = 0.5;
    attitude_cfg.yaw.ki = 0.05;
    attitude_cfg.yaw.kd = 0.05;

    attitude_cfg.yaw.output_min = -0.15;
    attitude_cfg.yaw.output_max = 0.15;

    AttitudeController attitude_controller(
        attitude_cfg);

    // -------------------------------------------------
    // Altitude controller
    // -------------------------------------------------
    AltitudeControllerConfig altitude_cfg;

    altitude_cfg.hover_collective =
        (params.mass * params.gravity) /
        max_total_thrust;

    altitude_cfg.collective_min = 0.0;
    altitude_cfg.collective_max = 1.0;

    altitude_cfg.altitude_pid.kp = 0.35;
    altitude_cfg.altitude_pid.ki = 0.08;
    altitude_cfg.altitude_pid.kd = 0.18;

    altitude_cfg.altitude_pid.output_min = -0.25;
    altitude_cfg.altitude_pid.output_max = 0.25;

    altitude_cfg.altitude_pid.integral_min = -1.0;
    altitude_cfg.altitude_pid.integral_max = 1.0;

    AltitudeController altitude_controller(
        altitude_cfg);

    // -------------------------------------------------
    // Mixer + actuator model
    // -------------------------------------------------
    MotorMixer mixer;

    ActuatorModelConfig actuator_cfg;

    actuator_cfg.arm_length = 0.25;
    actuator_cfg.max_motor_thrust = 10.0;
    actuator_cfg.yaw_torque_coefficient = 0.02;

    ActuatorModel actuator_model(
        actuator_cfg);

    // -------------------------------------------------
    // Mission setpoints
    //
    // Start: (0, 0, 1)
    // Target: (5, 3, 1)
    // -------------------------------------------------
    Position2D position_setpoint;

    position_setpoint.x = 5.0;
    position_setpoint.y = 3.0;

    constexpr double altitude_setpoint = 1.0;

    // -------------------------------------------------
    // Log
    // -------------------------------------------------
    std::ofstream log(
        "position_hold_6dof.csv");

    if (!log.is_open()) {
        std::cerr
            << "Failed to open position-control log.\n";
        return 1;
    }

    log
        << "time,"
        << "x_setpoint,y_setpoint,"
        << "x,y,z,"
        << "vx,vy,vz,"
        << "roll_setpoint,pitch_setpoint,"
        << "roll,pitch,yaw,"
        << "altitude_setpoint,"
        << "collective,"
        << "fl,fr,rl,rr\n";

    // -------------------------------------------------
    // Closed-loop simulation
    // -------------------------------------------------
    for (int step = 0;
         step < static_cast<int>(sim_time / dt);
         ++step) {

        const double time =
            step * dt;

        const auto& state =
            dynamics.state();

        const auto euler =
            dynamics.eulerAngles();

        // ----- Outer position loop -----

        Position2D position_measurement;

        position_measurement.x =
            state.position.x;

        position_measurement.y =
            state.position.y;

        const auto position_cmd =
            position_controller.update(
                position_setpoint,
                position_measurement,
                dt);

        // ----- Inner attitude loop -----

        Attitude attitude_setpoint;

        attitude_setpoint.roll =
            position_cmd.roll;

        attitude_setpoint.pitch =
            position_cmd.pitch;

        attitude_setpoint.yaw = 0.0;

        Attitude attitude_measurement;

        attitude_measurement.roll =
            euler.x;

        attitude_measurement.pitch =
            euler.y;

        attitude_measurement.yaw =
            euler.z;

        const auto attitude_cmd =
            attitude_controller.update(
                attitude_setpoint,
                attitude_measurement,
                dt);

        // ----- Altitude loop -----

        const double collective =
            altitude_controller.update(
                altitude_setpoint,
                state.position.z,
                dt);

        // ----- Mixer -----

        MixerInput mixer_input;

        mixer_input.collective =
            collective;

        mixer_input.roll =
            attitude_cmd.roll;

        mixer_input.pitch =
            attitude_cmd.pitch;

        mixer_input.yaw =
            attitude_cmd.yaw;

        const auto motors =
            mixer.mix(mixer_input);

        // ----- Actuator physics -----

        const auto forces =
            actuator_model.calculate(
                motors);

        // ----- 6-DOF plant -----

        Quadrotor6DOFInput input;

        input.total_thrust =
            forces.total_thrust;

        input.torque.x =
            forces.roll_torque;

        input.torque.y =
            forces.pitch_torque;

        input.torque.z =
            forces.yaw_torque;

        dynamics.update(input, dt);

        // ----- Log updated state -----

        const auto& updated =
            dynamics.state();

        const auto updated_euler =
            dynamics.eulerAngles();

        log
            << time << ","
            << position_setpoint.x << ","
            << position_setpoint.y << ","
            << updated.position.x << ","
            << updated.position.y << ","
            << updated.position.z << ","
            << updated.velocity.x << ","
            << updated.velocity.y << ","
            << updated.velocity.z << ","
            << attitude_setpoint.roll << ","
            << attitude_setpoint.pitch << ","
            << updated_euler.x << ","
            << updated_euler.y << ","
            << updated_euler.z << ","
            << altitude_setpoint << ","
            << collective << ","
            << motors.front_left << ","
            << motors.front_right << ","
            << motors.rear_left << ","
            << motors.rear_right
            << "\n";
    }

    // -------------------------------------------------
    // Results
    // -------------------------------------------------
    const auto& final_state =
        dynamics.state();

    const auto final_euler =
        dynamics.eulerAngles();

    const double position_error =
        std::sqrt(
            std::pow(
                position_setpoint.x -
                final_state.position.x,
                2.0) +
            std::pow(
                position_setpoint.y -
                final_state.position.y,
                2.0));

    std::cout
        << "6-DOF position-control mission complete.\n"
        << "Target X/Y: "
        << position_setpoint.x
        << " / "
        << position_setpoint.y
        << " m\n"
        << "Final X/Y: "
        << final_state.position.x
        << " / "
        << final_state.position.y
        << " m\n"
        << "Horizontal position error: "
        << position_error
        << " m\n"
        << "Final altitude: "
        << final_state.position.z
        << " m\n"
        << "Final roll/pitch: "
        << final_euler.x * 180.0 / M_PI
        << " / "
        << final_euler.y * 180.0 / M_PI
        << " deg\n"
        << "Log written to position_hold_6dof.csv\n";

    return 0;
}
