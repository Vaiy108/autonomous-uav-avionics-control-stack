#include "avionics/middleware/LocalMessageBus.hpp"

#include <utility>

namespace avionics::middleware
{

void LocalMessageBus::subscribeImu(ImuCallback callback)
{
    imu_subscribers_.push_back(std::move(callback));
}

void LocalMessageBus::subscribeGnss(GnssCallback callback)
{
    gnss_subscribers_.push_back(std::move(callback));
}

void LocalMessageBus::subscribeBarometer(BarometerCallback callback)
{
    barometer_subscribers_.push_back(
        std::move(callback));
}

void LocalMessageBus::subscribeMagnetometer(MagnetometerCallback callback)
{
    magnetometer_subscribers_.push_back(
        std::move(callback));
}

void LocalMessageBus::publish(
    const messages::ImuSample& sample)
{
    for (const auto& callback : imu_subscribers_)
    {
        callback(sample);
    }
}

void LocalMessageBus::publish(
    const messages::GnssSample& sample)
{
    for (const auto& callback : gnss_subscribers_)
    {
        callback(sample);
    }
}

void LocalMessageBus::publish(
    const messages::BarometerSample& sample)
{
    for (const auto& callback :barometer_subscribers_)
    {
        callback(sample);
    }
}

void LocalMessageBus::publish(
    const messages::MagnetometerSample& sample)
{
    for (const auto& callback :magnetometer_subscribers_)
    {
        callback(sample);
    }
}

}  // namespace avionics::middleware