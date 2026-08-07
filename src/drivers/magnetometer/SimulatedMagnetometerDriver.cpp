#include "avionics/drivers/SimulatedMagnetometerDriver.hpp"

namespace avionics::drivers
{

// Constructor
SimulatedMagnetometerDriver::SimulatedMagnetometerDriver(
    const platform::IClock& clock)
    : clock_{clock}
{
}

bool SimulatedMagnetometerDriver::initialize()
{
    
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
    

    /*
     * Simulate a slow change in vehicle heading.
     */
    magnetic_x_ut_ += 0.02F;
    magnetic_y_ut_ += 0.03F;
    magnetic_z_ut_ -= 0.01F;

    messages::MagnetometerSample sample{};
    
    sample.timestamp_us = clock_.nowUs();// shared Clock

    sample.magnetic_field_ut = {
        magnetic_x_ut_,
        magnetic_y_ut_,
        magnetic_z_ut_
    };

    sample.valid = true;

    return sample;
}

}  // namespace avionics::drivers
