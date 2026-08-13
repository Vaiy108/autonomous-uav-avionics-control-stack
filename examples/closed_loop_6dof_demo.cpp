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
    constexpr double sim_time = 10.0;

    // -------------------------------------------------
    // Vehicle parameters
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
    // Attitude controller
    // -------------------------------------------------
    AttitudeControllerConfig attitude_cfg;

    attitude_cfg.roll.kp = 0.8;
    attitude_cfg.roll.ki = 0.10;
    attitude_cfg.roll.kd = 0.08;

    attitude_cfg.roll.output_min = -0.20;
    attitude_cfg.roll.output_max = 0.20;

    attitude_cfg.pitch = attitude_cfg.roll;

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

    constexpr double max_total_thrust = 40.0;

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
    // Mixer and actuator model
    // -------------------------------------------------
    MotorMixer mixer;

    ActuatorModelConfig actuator_cfg;

    actuator_cfg.arm_length = 0.25;
    actuator_cfg.max_motor_thrust = 10.0;
    actuator_cfg.yaw_torque_coefficient = 0.02;

    ActuatorModel actuator_model(
        actuator_cfg);

    // -------------------------------------------------
    // Setpoints
    // -------------------------------------------------
    Attitude attitude_setpoint;

    attitude_setpoint.roll =
        15.0 * M_PI / 180.0;

    attitude_setpoint.pitch = 0.0;
    attitude_setpoint.yaw = 0.0;

    constexpr double altitude_setpoint = 1.0;

    // -------------------------------------------------
    // Logging
    // -------------------------------------------------
    std::ofstream log(
        "closed_loop_6dof.csv");

    if (!log.is_open()) {
        std::cerr
            << "Failed to open 6-DOF log.\n";

        return 1;
    }

    log
        << "time,"
        << "roll_setpoint,roll,"
        << "pitch,yaw,"
        << "altitude_setpoint,altitude,"
        << "x,y,"
        << "vx,vy,vz,"
        << "collective,"
        << "fl,fr,rl,rr\n";

    // -------------------------------------------------
    // Closed-loop simulation
    // -------------------------------------------------
    for (int step = 0;
         step <
             static_cast<int>(
                 sim_time / dt);
         ++step) {

        const double time =
            step * dt;

        const auto euler =
            dynamics.eulerAngles();

        const auto& state =
            dynamics.state();

        Attitude measurement;

        measurement.roll = euler.x;
        measurement.pitch = euler.y;
        measurement.yaw = euler.z;

        const auto attitude_cmd =
            attitude_controller.update(
                attitude_setpoint,
                measurement,
                dt);

        const double collective =
            altitude_controller.update(
                altitude_setpoint,
                state.position.z,
                dt);

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
            mixer.mix(
                mixer_input);

        const auto forces =
            actuator_model.calculate(
                motors);

        Quadrotor6DOFInput input;

        input.total_thrust =
            forces.total_thrust;

        input.torque.x =
            forces.roll_torque;

        input.torque.y =
            forces.pitch_torque;

        input.torque.z =
            forces.yaw_torque;

        dynamics.update(
            input,
            dt);

        const auto updated_euler =
            dynamics.eulerAngles();

        const auto& updated =
            dynamics.state();

        log
            << time << ","
            << attitude_setpoint.roll << ","
            << updated_euler.x << ","
            << updated_euler.y << ","
            << updated_euler.z << ","
            << altitude_setpoint << ","
            << updated.position.z << ","
            << updated.position.x << ","
            << updated.position.y << ","
            << updated.velocity.x << ","
            << updated.velocity.y << ","
            << updated.velocity.z << ","
            << collective << ","
            << motors.front_left << ","
            << motors.front_right << ","
            << motors.rear_left << ","
            << motors.rear_right
            << "\n";
    }

    const auto final_euler =
        dynamics.eulerAngles();

    const auto& final_state =
        dynamics.state();

    std::cout
        << "6-DOF closed-loop simulation complete.\n"
        << "Final roll: "
        << final_euler.x *
               180.0 / M_PI
        << " deg\n"
        << "Final pitch: "
        << final_euler.y *
               180.0 / M_PI
        << " deg\n"
        << "Final yaw: "
        << final_euler.z *
               180.0 / M_PI
        << " deg\n"
        << "Final altitude: "
        << final_state.position.z
        << " m\n"
        << "Final position X/Y: "
        << final_state.position.x
        << " / "
        << final_state.position.y
        << " m\n"
        << "Log written to "
           "closed_loop_6dof.csv\n";

    return 0;
}
