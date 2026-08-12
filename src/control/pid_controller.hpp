#pragma once

#include <cstdint>

namespace flight_control {

struct PIDConfig {
    double kp{0.0};
    double ki{0.0};
    double kd{0.0};

    double output_min{-1.0};
    double output_max{1.0};

    double integral_min{-1.0};
    double integral_max{1.0};

    // First-order low-pass coefficient for derivative term.
    // 0.0 -> no derivative response
    // 1.0 -> unfiltered derivative
    double derivative_filter_alpha{0.2};

    // Valid controller timestep limits [s].
    double dt_min{1e-4};
    double dt_max{0.1};
};

class PIDController {
public:
    explicit PIDController(const PIDConfig& config);

    /**
     * @brief Calculate one PID control update.
     *
     * @param setpoint Desired state.
     * @param measurement Measured state.
     * @param dt Controller timestep [s].
     * @return Saturated control command.
     */
    double update(double setpoint, double measurement, double dt);

    void reset();

    void setConfig(const PIDConfig& config);
    const PIDConfig& config() const noexcept;

    double integralState() const noexcept;
    double previousError() const noexcept;
    double derivativeState() const noexcept;
    bool initialized() const noexcept;

private:
    static double clamp(double value, double minimum, double maximum);
    static bool finite(double value) noexcept;

    void validateConfig(const PIDConfig& config) const;

    PIDConfig config_;

    double integral_{0.0};
    double previous_error_{0.0};
    double filtered_derivative_{0.0};

    bool initialized_{false};
};

}  // namespace flight_control
