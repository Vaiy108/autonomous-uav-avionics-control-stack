#include "pid_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace flight_control {

PIDController::PIDController(const PIDConfig& config)
    : config_(config) {
    validateConfig(config_);
}

double PIDController::update(
    const double setpoint,
    const double measurement,
    const double dt) {

    if (!finite(setpoint) || !finite(measurement) || !finite(dt)) {
        return 0.0;
    }

    if (dt < config_.dt_min || dt > config_.dt_max) {
        return 0.0;
    }

    const double error = setpoint - measurement;

    // Proportional term.
    const double proportional = config_.kp * error;

    // Derivative-on-error with first-order low-pass filtering.
    double raw_derivative = 0.0;

    if (initialized_) {
        raw_derivative = (error - previous_error_) / dt;
    }

    filtered_derivative_ =
        config_.derivative_filter_alpha * raw_derivative +
        (1.0 - config_.derivative_filter_alpha) * filtered_derivative_;

    const double derivative = config_.kd * filtered_derivative_;

    // Candidate integral state.
    double candidate_integral =
        integral_ + error * dt;

    candidate_integral = clamp(
        candidate_integral,
        config_.integral_min,
        config_.integral_max);

    const double integral_term =
        config_.ki * candidate_integral;

    const double unsaturated_output =
        proportional + integral_term + derivative;

    const double saturated_output =
        clamp(
            unsaturated_output,
            config_.output_min,
            config_.output_max);

    /*
     * Conditional-integration anti-windup.
     *
     * Allow integration when:
     *  - output is not saturated, or
     *  - the error would drive the controller away from saturation.
     */
    const bool saturated_high =
        unsaturated_output > config_.output_max;

    const bool saturated_low =
        unsaturated_output < config_.output_min;

    const bool integrate =
        (!saturated_high && !saturated_low) ||
        (saturated_high && error < 0.0) ||
        (saturated_low && error > 0.0);

    if (integrate) {
        integral_ = candidate_integral;
    }

    previous_error_ = error;
    initialized_ = true;

    return saturated_output;
}

void PIDController::reset() {
    integral_ = 0.0;
    previous_error_ = 0.0;
    filtered_derivative_ = 0.0;
    initialized_ = false;
}

void PIDController::setConfig(const PIDConfig& config) {
    validateConfig(config);
    config_ = config;
    reset();
}

const PIDConfig& PIDController::config() const noexcept {
    return config_;
}

double PIDController::integralState() const noexcept {
    return integral_;
}

double PIDController::previousError() const noexcept {
    return previous_error_;
}

double PIDController::derivativeState() const noexcept {
    return filtered_derivative_;
}

bool PIDController::initialized() const noexcept {
    return initialized_;
}

double PIDController::clamp(
    const double value,
    const double minimum,
    const double maximum) {

    return std::clamp(value, minimum, maximum);
}

bool PIDController::finite(const double value) noexcept {
    return std::isfinite(value);
}

void PIDController::validateConfig(
    const PIDConfig& config) const {

    if (!finite(config.kp) ||
        !finite(config.ki) ||
        !finite(config.kd)) {
        throw std::invalid_argument(
            "PID gains must be finite");
    }

    if (config.output_min >= config.output_max) {
        throw std::invalid_argument(
            "PID output limits are invalid");
    }

    if (config.integral_min >= config.integral_max) {
        throw std::invalid_argument(
            "PID integral limits are invalid");
    }

    if (config.derivative_filter_alpha < 0.0 ||
        config.derivative_filter_alpha > 1.0) {
        throw std::invalid_argument(
            "Derivative filter alpha must be in [0, 1]");
    }

    if (config.dt_min <= 0.0 ||
        config.dt_min >= config.dt_max) {
        throw std::invalid_argument(
            "PID timestep limits are invalid");
    }
}

}  // namespace flight_control
