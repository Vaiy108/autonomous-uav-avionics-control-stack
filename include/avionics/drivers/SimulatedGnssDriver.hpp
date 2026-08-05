#pragma once

#include "avionics/drivers/ISensorDriver.hpp"
#include "avionics/messages/SensorMessages.hpp"
#include "avionics/platform/interfaces/IClock.hpp"

namespace avionics::drivers
{

class SimulatedGnssDriver final : public ISensorDriver
{
public:
     explicit SimulatedGnssDriver(const platform::IClock& clock);
     
    bool initialize() override;
    bool selfTest() override;

    [[nodiscard]] SensorStatus status() const override;
    [[nodiscard]] std::string_view name() const override;

    [[nodiscard]] messages::GnssSample read();

private:
    const platform::IClock& clock_;

    SensorStatus status_{SensorStatus::uninitialized};
    
    double latitude_deg_{52.2689};
    double longitude_deg_{10.5268};
    float altitude_m_{78.0F};
};

}  // namespace avionics::drivers