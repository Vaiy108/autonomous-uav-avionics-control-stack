#pragma once

#include "avionics/messages/SensorMessages.hpp"

#include <functional>

namespace avionics::middleware
{

using ImuCallback =
    std::function<void(const messages::ImuSample&)>;

using GnssCallback =
    std::function<void(const messages::GnssSample&)>;

using BarometerCallback =
    std::function<void(const messages::BarometerSample&)>;

using MagnetometerCallback =
    std::function<void(const messages::MagnetometerSample&)>;

class IMessageBus
{
public:
    virtual ~IMessageBus() = default;

    virtual void subscribeImu(ImuCallback callback) = 0;
    virtual void subscribeGnss(GnssCallback callback) = 0;
    virtual void subscribeBarometer(BarometerCallback callback) = 0;
    virtual void subscribeMagnetometer(MagnetometerCallback callback) = 0;

    virtual void publish(
        const messages::ImuSample& sample) = 0;

    virtual void publish(
        const messages::GnssSample& sample) = 0;

    virtual void publish(
        const messages::BarometerSample& sample) = 0;

    virtual void publish(
        const messages::MagnetometerSample& sample) = 0;
};

}  // namespace avionics::middleware