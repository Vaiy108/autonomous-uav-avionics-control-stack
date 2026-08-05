#pragma once

#include "avionics/platform/interfaces/IClock.hpp"

namespace avionics::platform
{

class SimulationClock final : public IClock
{
public:
    [[nodiscard]] messages::TimestampUs nowUs() const override;

    void setTimeUs(messages::TimestampUs timestamp_us);
    void advanceUs(messages::TimestampUs delta_us);

private:
    messages::TimestampUs current_time_us_{0};
};

}  // namespace avionics::platform