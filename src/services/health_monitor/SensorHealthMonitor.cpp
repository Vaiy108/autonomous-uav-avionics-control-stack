#include "avionics/services/SensorHealthMonitor.hpp"

namespace avionics::services
{

bool SystemHealth::allHealthy() const
{
    return imu.state == HealthState::healthy &&
           gnss.state == HealthState::healthy &&
           barometer.state == HealthState::healthy &&
           magnetometer.state == HealthState::healthy;
}

bool SystemHealth::degraded() const
{
    return !allHealthy();
}

SensorHealthMonitor::SensorHealthMonitor(
    const middleware::SensorManager& sensor_manager)
    : sensor_manager_{sensor_manager}
{
}

SystemHealth SensorHealthMonitor::evaluate(
    const messages::TimestampUs current_time_us) const
{
    SystemHealth health{};

    health.imu = evaluateSensor(
        sensor_manager_.hasImuSample(),
        sensor_manager_.hasImuSample()
            ? sensor_manager_.latestImu().timestamp_us
            : 0,
        current_time_us,
        ImuStaleThresholdUs);

    health.gnss = evaluateSensor(
        sensor_manager_.hasGnssSample(),
        sensor_manager_.hasGnssSample()
            ? sensor_manager_.latestGnss().timestamp_us
            : 0,
        current_time_us,
        GnssStaleThresholdUs);

    health.barometer = evaluateSensor(
        sensor_manager_.hasBarometerSample(),
        sensor_manager_.hasBarometerSample()
            ? sensor_manager_.latestBarometer().timestamp_us
            : 0,
        current_time_us,
        BarometerStaleThresholdUs);

    health.magnetometer = evaluateSensor(
        sensor_manager_.hasMagnetometerSample(),
        sensor_manager_.hasMagnetometerSample()
            ? sensor_manager_.latestMagnetometer().timestamp_us
            : 0,
        current_time_us,
        MagnetometerStaleThresholdUs);

    return health;
}

std::string_view SensorHealthMonitor::toString(
    const HealthState state)
{
    switch (state)
    {
        case HealthState::unavailable:
            return "UNAVAILABLE";

        case HealthState::healthy:
            return "HEALTHY";

        case HealthState::stale:
            return "STALE";
    }

    return "UNKNOWN";
}

SensorHealth SensorHealthMonitor::evaluateSensor(
    const bool sample_available,
    const messages::TimestampUs sample_timestamp_us,
    const messages::TimestampUs current_time_us,
    const messages::TimestampUs stale_threshold_us)
{
    if (!sample_available)
    {
        return {
            HealthState::unavailable,
            0
        };
    }

    /*
     * A future timestamp indicates inconsistent timing. Treat it as stale
     * for this initial monitor rather than allowing unsigned underflow.
     */
    if (sample_timestamp_us > current_time_us)
    {
        return {
            HealthState::stale,
            0
        };
    }

    const auto sample_age_us =
        current_time_us - sample_timestamp_us;

    if (sample_age_us > stale_threshold_us)
    {
        return {
            HealthState::stale,
            sample_age_us
        };
    }

    return {
        HealthState::healthy,
        sample_age_us
    };
}

}  // namespace avionics::services