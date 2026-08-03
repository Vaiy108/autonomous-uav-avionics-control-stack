#include "avionics/drivers/SimulatedMagnetometerDriver.hpp"

namespace avionics::drivers
{

bool SimulatedMagnetometerDriver::initialize()
{
    timestamp_us_ = 0;
    status_ = SensorStatus::ready;

    return true;
}

bool SimulatedMagnetometerDriver::selfTest()
{
    return status_ == SensorStatus::ready;
}

SensorStatus SimulatedMagnetometerDriver::status() const
{
    return status_;
}

std::string_view SimulatedMagnetometerDriver::name() const
{
    return "Simulated Magnetometer";
}

messages::MagnetometerSample SimulatedMagnetometerDriver::read()
{
    if (status_ != SensorStatus::ready)
    {
        return {};
    }

    /*
     * Magnetometer output rate: 50 Hz
     * 20,000 microseconds = 20 milliseconds.
     */
    timestamp_us_ += 20'000;

    /*
     * Simulate a slow change in vehicle heading.
     */
    magnetic_x_ut_ += 0.02F;
    magnetic_y_ut_ += 0.03F;
    magnetic_z_ut_ -= 0.01F;

    messages::MagnetometerSample sample{};
    sample.timestamp_us = timestamp_us_;

    sample.magnetic_field_ut = {
        magnetic_x_ut_,
        magnetic_y_ut_,
        magnetic_z_ut_
    };

    sample.valid = true;

    return sample;
}

}  // namespace avionics::drivers