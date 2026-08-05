#pragma once

#include "avionics/drivers/ISensorDriver.hpp"
#include "avionics/messages/SensorMessages.hpp"
#include "avionics/platform/interfaces/IClock.hpp"

namespace avionics::drivers
{

class SimulatedBarometerDriver final : public ISensorDriver
{
public:
    explicit SimulatedBarometerDriver(const platform::IClock& clock);

    bool initialize() override;
    bool selfTest() override;

    [[nodiscard]] SensorStatus status() const override;
    [[nodiscard]] std::string_view name() const override;

    [[nodiscard]] messages::BarometerSample read();

private:
    const platform::IClock& clock_;
    
    SensorStatus status_{SensorStatus::uninitialized};
    

    float pressure_pa_{100'400.0F};
    float temperature_c_{21.0F};
    float altitude_m_{78.0F};
};

}  // namespace avionics::drivers