#pragma once

#include "avionics/messages/SensorMessages.hpp"
#include "avionics/middleware/IMessageBus.hpp"

namespace avionics::middleware
{

class SensorManager
{
public:
    explicit SensorManager(IMessageBus& message_bus);

    void start();

    [[nodiscard]] bool hasImuSample() const;
    [[nodiscard]] bool hasGnssSample() const;
    [[nodiscard]] bool hasBarometerSample() const;
    [[nodiscard]] bool hasMagnetometerSample() const;

    [[nodiscard]] const messages::ImuSample& latestImu() const;
    [[nodiscard]] const messages::GnssSample& latestGnss() const;
    [[nodiscard]] const messages::BarometerSample& latestBarometer() const;
    [[nodiscard]] const messages::MagnetometerSample&
    latestMagnetometer() const;

    [[nodiscard]] bool allSensorsAvailable() const;

private:
    void handleImu(const messages::ImuSample& sample);
    void handleGnss(const messages::GnssSample& sample);
    void handleBarometer(
        const messages::BarometerSample& sample);
    void handleMagnetometer(
        const messages::MagnetometerSample& sample);

    IMessageBus& message_bus_;

    messages::ImuSample latest_imu_{};
    messages::GnssSample latest_gnss_{};
    messages::BarometerSample latest_barometer_{};
    messages::MagnetometerSample latest_magnetometer_{};

    bool has_imu_{false};
    bool has_gnss_{false};
    bool has_barometer_{false};
    bool has_magnetometer_{false};
    bool started_{false};
};

}  // namespace avionics::middleware