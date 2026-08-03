#include "avionics/drivers/SimulatedGnssDriver.hpp"

namespace avionics::drivers
{

bool SimulatedGnssDriver::initialize()
{
    timestamp_us_ = 0;
    status_ = SensorStatus::ready;

    return true;
}

bool SimulatedGnssDriver::selfTest()
{
    return status_ == SensorStatus::ready;
}

SensorStatus SimulatedGnssDriver::status() const
{
    return status_;
}

std::string_view SimulatedGnssDriver::name() const
{
    return "Simulated GNSS";
}

messages::GnssSample SimulatedGnssDriver::read()
{
    if (status_ != SensorStatus::ready)
    {
        return {};
    }

    /*
     * GNSS output rate: 10 Hz
     * 100,000 microseconds = 100 milliseconds.
     */
    timestamp_us_ += 100'000;

    /*
     * Simulate slow motion by incrementing the coordinates slightly
     * on every sample.
     */
    latitude_deg_ += 0.000001;
    longitude_deg_ += 0.000002;
    altitude_m_ += 0.01F;

    messages::GnssSample sample{};
    sample.timestamp_us = timestamp_us_;

    sample.latitude_deg = latitude_deg_;
    sample.longitude_deg = longitude_deg_;
    sample.altitude_m = altitude_m_;

    sample.ground_speed_mps = 5.2F;
    sample.valid = true;

    return sample;
}

}  // namespace avionics::drivers