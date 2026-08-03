#pragma once

#include "avionics/drivers/ISensorDriver.hpp"
#include "avionics/messages/SensorMessages.hpp"

namespace avionics::drivers
{

class SimulatedMagnetometerDriver final : public ISensorDriver
{
public:
    bool initialize() override;
    bool selfTest() override;

    [[nodiscard]] SensorStatus status() const override;
    [[nodiscard]] std::string_view name() const override;

    [[nodiscard]] messages::MagnetometerSample read();

private:
    SensorStatus status_{SensorStatus::uninitialized};
    messages::TimestampUs timestamp_us_{0};

    float magnetic_x_ut_{21.0F};
    float magnetic_y_ut_{2.0F};
    float magnetic_z_ut_{43.0F};
};

}  // namespace avionics::drivers