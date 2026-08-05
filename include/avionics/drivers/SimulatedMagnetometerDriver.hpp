#pragma once

#include "avionics/drivers/ISensorDriver.hpp"
#include "avionics/messages/SensorMessages.hpp"
#include "avionics/platform/interfaces/IClock.hpp"

namespace avionics::drivers
{

class SimulatedMagnetometerDriver final : public ISensorDriver
{
public:
    explicit SimulatedMagnetometerDriver(const platform::IClock& clock);
    bool initialize() override;
    bool selfTest() override;

    [[nodiscard]] SensorStatus status() const override;
    [[nodiscard]] std::string_view name() const override;

    [[nodiscard]] messages::MagnetometerSample read();

private:
    const platform::IClock& clock_;

    SensorStatus status_{SensorStatus::uninitialized};
   

    float magnetic_x_ut_{21.0F};
    float magnetic_y_ut_{2.0F};
    float magnetic_z_ut_{43.0F};
};

}  // namespace avionics::drivers