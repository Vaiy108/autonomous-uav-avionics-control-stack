#include "attitude_controller.hpp"
#include "altitude_controller.hpp"
#include "motor_mixer.hpp"
#include "actuator_model.hpp"
#include "quadrotor_dynamics.hpp"

#include <cmath>
#include <fstream>
#include <iostream>

using namespace flight_control;

int main() {
    constexpr double dt = 0.01;
    constexpr double sim_time = 8.0;

    // ----- Attitude controller -----
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

    AttitudeController attitude_controller(attitude_cfg);

    // ----- Vehicle parameters -----
    QuadrotorParameters params;
    params.mass = 2.0;
    params.inertia_xx = 0.03;
    params.inertia_yy = 0.03;
    params.inertia_zz = 0.05;
    params.max_total_thrust = 40.0;

    // ----- Altitude controller -----
    AltitudeControllerConfig altitude_cfg;

    altitude_cfg.hover_collective =
        (params.mass * params.gravity) /
        params.max_total_thrust;

    altitude_cfg.collective_min = 0.0;
    altitude_cfg.collective_max = 1.0;

    altitude_cfg.altitude_pid.kp = 0.35;
    altitude_cfg.altitude_pid.ki = 0.08;
    altitude_cfg.altitude_pid.kd = 0.18;

    altitude_cfg.altitude_pid.output_min = -0.25;
    altitude_cfg.altitude_pid.output_max = 0.25;

    altitude_cfg.altitude_pid.integral_min = -1.0;
    altitude_cfg.altitude_pid.integral_max = 1.0;

    AltitudeController altitude_controller(altitude_cfg);

    // ----- Mixer / actuator -----
    MotorMixer mixer;

    ActuatorModelConfig actuator_cfg;
    actuator_cfg.arm_length = 0.25;
    actuator_cfg.max_motor_thrust = 10.0;
    actuator_cfg.yaw_torque_coefficient = 0.02;

    ActuatorModel actuator_model(actuator_cfg);

    // ----- Dynamics -----
    QuadrotorDynamics dynamics(params);

    QuadrotorState initial_state;
    initial_state.altitude = 1.0;
    dynamics.setState(initial_state);

    // ----- Setpoints -----
    Attitude attitude_setpoint;
    attitude_setpoint.roll = 15.0 * M_PI / 180.0;
    attitude_setpoint.pitch = 0.0;
    attitude_setpoint.yaw = 0.0;

    constexpr double altitude_setpoint = 1.0;

    // ----- Log -----
    std::ofstream log("closed_loop_attitude.csv");

    if (!log.is_open()) {
        std::cerr << "Failed to open output log.\n";
        return 1;
    }

    log << "time,"
           "roll_setpoint,roll,"
           "pitch,yaw,"
           "roll_command,"
           "collective,"
           "fl,fr,rl,rr,"
           "altitude_setpoint,altitude\n";

    for (int step = 0;
         step < static_cast<int>(sim_time / dt);
         ++step) {

        const double time = step * dt;

        const auto& state = dynamics.state();

        Attitude measurement;
        measurement.roll = state.roll;
        measurement.pitch = state.pitch;
        measurement.yaw = state.yaw;

        const BodyTorqueCommand attitude_cmd =
            attitude_controller.update(
                attitude_setpoint,
                measurement,
                dt);

        const double collective =
            altitude_controller.update(
                altitude_setpoint,
                state.altitude,
                dt);

        MixerInput mixer_input;
        mixer_input.collective = collective;
        mixer_input.roll = attitude_cmd.roll;
        mixer_input.pitch = attitude_cmd.pitch;
        mixer_input.yaw = attitude_cmd.yaw;

        const MotorCommands motors =
            mixer.mix(mixer_input);

        const ActuatorForces forces =
            actuator_model.calculate(motors);

        QuadrotorInput input;
        input.roll_torque = forces.roll_torque;
        input.pitch_torque = forces.pitch_torque;
        input.yaw_torque = forces.yaw_torque;
        input.collective_thrust =
            forces.total_thrust /
            params.max_total_thrust;

        dynamics.update(input, dt);

        const auto& updated = dynamics.state();

        log
            << time << ","
            << attitude_setpoint.roll << ","
            << updated.roll << ","
            << updated.pitch << ","
            << updated.yaw << ","
            << attitude_cmd.roll << ","
            << collective << ","
            << motors.front_left << ","
            << motors.front_right << ","
            << motors.rear_left << ","
            << motors.rear_right << ","
            << altitude_setpoint << ","
            << updated.altitude << "\n";
    }

    const auto& final_state = dynamics.state();

    std::cout
        << "Closed-loop simulation complete.\n"
        << "Final roll: "
        << final_state.roll * 180.0 / M_PI
        << " deg\n"
        << "Final altitude: "
        << final_state.altitude
        << " m\n"
        << "Log written to closed_loop_attitude.csv\n";

    return 0;
}
