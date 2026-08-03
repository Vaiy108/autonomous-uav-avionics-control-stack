#pragma once

#include "avionics/messages/SensorMessages.hpp"
#include "avionics/middleware/SensorManager.hpp"

#include <string_view>

namespace avionics::services
{

enum class HealthState
{
    unavailable,
    healthy,
    stale
};

struct SensorHealth
{
    HealthState state{HealthState::unavailable};
    messages::TimestampUs sample_age_us{};
};

struct SystemHealth
{
    SensorHealth imu{};
    SensorHealth gnss{};
    SensorHealth barometer{};
    SensorHealth magnetometer{};

    [[nodiscard]] bool allHealthy() const;
    [[nodiscard]] bool degraded() const;
};

class SensorHealthMonitor
{
public:
    explicit SensorHealthMonitor(
        const middleware::SensorManager& sensor_manager);

    [[nodiscard]] SystemHealth evaluate(
        messages::TimestampUs current_time_us) const;

    [[nodiscard]] static std::string_view toString(
        HealthState state);

private:
    [[nodiscard]] static SensorHealth evaluateSensor(
        bool sample_available,
        messages::TimestampUs sample_timestamp_us,
        messages::TimestampUs current_time_us,
        messages::TimestampUs stale_threshold_us);

    const middleware::SensorManager& sensor_manager_;

    static constexpr messages::TimestampUs
        ImuStaleThresholdUs{50'000};

    static constexpr messages::TimestampUs
        MagnetometerStaleThresholdUs{100'000};

    static constexpr messages::TimestampUs
        BarometerStaleThresholdUs{200'000};

    static constexpr messages::TimestampUs
        GnssStaleThresholdUs{500'000};
};

}  // namespace avionics::services