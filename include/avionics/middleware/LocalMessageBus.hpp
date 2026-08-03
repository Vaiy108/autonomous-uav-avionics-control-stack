#pragma once

#include "avionics/middleware/IMessageBus.hpp"

#include <vector>

namespace avionics::middleware
{

class LocalMessageBus final : public IMessageBus
{
public:
    void subscribeImu(ImuCallback callback) override;
    void subscribeGnss(GnssCallback callback) override;
    void subscribeBarometer(BarometerCallback callback) override;
    void subscribeMagnetometer(MagnetometerCallback callback) override;

    void publish(
        const messages::ImuSample& sample) override;

    void publish(
        const messages::GnssSample& sample) override;

    void publish(
        const messages::BarometerSample& sample) override;

    void publish(
        const messages::MagnetometerSample& sample) override;

private:
    std::vector<ImuCallback> imu_subscribers_{};
    std::vector<GnssCallback> gnss_subscribers_{};
    std::vector<BarometerCallback> barometer_subscribers_{};
    std::vector<MagnetometerCallback> magnetometer_subscribers_{};
};

}  // namespace avionics::middleware