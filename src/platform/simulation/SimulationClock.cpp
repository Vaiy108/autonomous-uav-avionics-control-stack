#include "avionics/platform/simulation/SimulationClock.hpp"

namespace avionics::platform
{

messages::TimestampUs SimulationClock::nowUs() const
{
    return current_time_us_;
}

void SimulationClock::setTimeUs(
    const messages::TimestampUs timestamp_us)
{
    current_time_us_ = timestamp_us;
}

void SimulationClock::advanceUs(
    const messages::TimestampUs delta_us)
{
    current_time_us_ += delta_us;
}

}  // namespace avionics::platform