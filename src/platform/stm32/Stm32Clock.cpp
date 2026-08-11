#include "avionics/platform/stm32/Stm32Clock.hpp"

extern "C"
{
#include "stm32f4xx_hal.h"
}

namespace avionics::platform
{

std::uint64_t Stm32Clock::nowUs() const
{
    return static_cast<std::uint64_t>(HAL_GetTick()) * 1000ULL;
}

}  // namespace avionics::platform
