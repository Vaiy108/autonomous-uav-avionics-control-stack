#pragma once

#include "avionics/drivers/ISensorDriver.hpp"
#include "avionics/messages/SensorMessages.hpp"
#include "avionics/platform/interfaces/IClock.hpp"

namespace avionics::drivers
{

class SimulatedImuDriver final : public ISensorDriver
{
public:
	explicit SimulatedImuDriver(const platform::IClock& clock);

    bool initialize() override;
    bool selfTest() override;

    [[nodiscard]] SensorStatus status() const override;
    [[nodiscard]] std::string_view name() const override;

    [[nodiscard]] messages::ImuSample read();

private:
	const platform::IClock& clock_
    SensorStatus status_{SensorStatus::uninitialized};
};

}  // namespace avionics::drivers