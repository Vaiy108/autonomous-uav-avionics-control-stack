#pragma once

#include "avionics/messages/SensorMessages.hpp"

namespace avionics::platform
{

class IClock
{
public:
    virtual ~IClock() = default;

    [[nodiscard]] virtual messages::TimestampUs nowUs() const = 0;
};

}  // namespace avionics::platform