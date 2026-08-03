#include "avionics/drivers/SimulatedBarometerDriver.hpp"

namespace avionics::drivers
{

bool SimulatedBarometerDriver::initialize()
{
    timestamp_us_ = 0;
    status_ = SensorStatus::ready;

    return true;
}

bool SimulatedBarometerDriver::selfTest()
{
    return status_ == SensorStatus::ready;
}

SensorStatus SimulatedBarometerDriver::status() const
{
    return status_;
}

std::string_view SimulatedBarometerDriver::name() const
{
    return "Simulated Barometer";
}

messages::BarometerSample SimulatedBarometerDriver::read()
{
    if (status_ != SensorStatus::ready)
    {
        return {};
    }

    /*
     * Barometer output rate: 25 Hz
     * 40,000 microseconds = 40 milliseconds.
     */
    timestamp_us_ += 40'000;

    /*
     * Simulate a slow climb:
     * altitude increases while pressure decreases slightly.
     */
    altitude_m_ += 0.04F;
    pressure_pa_ -= 0.45F;
    temperature_c_ += 0.001F;

    messages::BarometerSample sample{};
    sample.timestamp_us = timestamp_us_;

    sample.pressure_pa = pressure_pa_;
    sample.temperature_c = temperature_c_;
    sample.altitude_m = altitude_m_;

    sample.valid = true;

    return sample;
}

}  // namespace avionics::drivers