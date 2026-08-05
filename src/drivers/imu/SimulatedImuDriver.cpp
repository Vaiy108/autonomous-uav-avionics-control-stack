#include "avionics/drivers/SimulatedImuDriver.hpp"

namespace avionics::drivers
{
// Constructor
SimulatedImuDriver::SimulatedImuDriver(
    const platform::IClock& clock)
    : clock_{clock}
{
}

bool SimulatedImuDriver::initialize()
{
    status_ = SensorStatus::ready;

    return true;
}

bool SimulatedImuDriver::selfTest()
{
    return status_ == SensorStatus::ready;
}

SensorStatus SimulatedImuDriver::status() const
{
    return status_;
}

std::string_view SimulatedImuDriver::name() const
{
    return "Simulated IMU";
}

messages::ImuSample SimulatedImuDriver::read()
{
    if (status_ != SensorStatus::ready)
    {
        return {};
    }


    messages::ImuSample sample{};
    sample.timestamp_us = clock_.nowUs();

    sample.acceleration_mps2 = {
        0.02F,
        -0.01F,
        9.81F
    };

    sample.angular_velocity_rps = {
        0.001F,
        -0.002F,
        0.003F
    };

    sample.temperature_c = 25.0F;
    sample.valid = true;

    return sample;
}

}  // namespace avionics::drivers