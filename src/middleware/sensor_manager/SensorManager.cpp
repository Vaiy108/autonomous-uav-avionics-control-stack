#include "avionics/middleware/SensorManager.hpp"

namespace avionics::middleware
{

SensorManager::SensorManager(IMessageBus& message_bus)
    : message_bus_{message_bus}
{
}

void SensorManager::start()
{
    if (started_)
    {
        return;
    }

    message_bus_.subscribeImu(
        [this](const messages::ImuSample& sample)
        {
            handleImu(sample);
        });

    message_bus_.subscribeGnss(
        [this](const messages::GnssSample& sample)
        {
            handleGnss(sample);
        });

    message_bus_.subscribeBarometer(
        [this](const messages::BarometerSample& sample)
        {
            handleBarometer(sample);
        });

    message_bus_.subscribeMagnetometer(
        [this](const messages::MagnetometerSample& sample)
        {
            handleMagnetometer(sample);
        });

    started_ = true;
}

bool SensorManager::hasImuSample() const
{
    return has_imu_;
}

bool SensorManager::hasGnssSample() const
{
    return has_gnss_;
}

bool SensorManager::hasBarometerSample() const
{
    return has_barometer_;
}

bool SensorManager::hasMagnetometerSample() const
{
    return has_magnetometer_;
}

const messages::ImuSample& SensorManager::latestImu() const
{
    return latest_imu_;
}

const messages::GnssSample& SensorManager::latestGnss() const
{
    return latest_gnss_;
}

const messages::BarometerSample&
SensorManager::latestBarometer() const
{
    return latest_barometer_;
}

const messages::MagnetometerSample&
SensorManager::latestMagnetometer() const
{
    return latest_magnetometer_;
}

bool SensorManager::allSensorsAvailable() const
{
    return has_imu_ &&
           has_gnss_ &&
           has_barometer_ &&
           has_magnetometer_;
}

void SensorManager::handleImu(
    const messages::ImuSample& sample)
{
    if (!sample.valid)
    {
        return;
    }

    latest_imu_ = sample;
    has_imu_ = true;
}

void SensorManager::handleGnss(
    const messages::GnssSample& sample)
{
    if (!sample.valid)
    {
        return;
    }

    latest_gnss_ = sample;
    has_gnss_ = true;
}

void SensorManager::handleBarometer(
    const messages::BarometerSample& sample)
{
    if (!sample.valid)
    {
        return;
    }

    latest_barometer_ = sample;
    has_barometer_ = true;
}

void SensorManager::handleMagnetometer(
    const messages::MagnetometerSample& sample)
{
    if (!sample.valid)
    {
        return;
    }

    latest_magnetometer_ = sample;
    has_magnetometer_ = true;
}

}  // namespace avionics::middleware