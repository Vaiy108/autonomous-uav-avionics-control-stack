#pragma once

#include "avionics/platform/interfaces/IClock.hpp"

namespace avionics::platform
{

class Stm32Clock final : public IClock
{
public:
    [[nodiscard]] std::uint64_t nowUs() const override;
};

}  // namespace avionics::platform
