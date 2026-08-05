#include "avionics/platform/simulation/SimulatedSpiBus.hpp"

namespace avionics::platform
{

bool SimulatedSpiBus::open()
{
    open_ = true;
    return true;
}

void SimulatedSpiBus::close()
{
    open_ = false;
}

bool SimulatedSpiBus::isOpen() const
{
    return open_;
}

bool SimulatedSpiBus::writeRegister(
    const std::uint8_t register_address,
    const std::uint8_t value)
{
    if (!open_)
    {
        return false;
    }

    registers_[register_address] = value;
    return true;
}

bool SimulatedSpiBus::readRegister(
    const std::uint8_t register_address,
    std::uint8_t& value)
{
    if (!open_)
    {
        return false;
    }

    value = registers_[register_address];
    return true;
}

bool SimulatedSpiBus::readRegisters(
    const std::uint8_t start_address,
    std::uint8_t* destination,
    const std::size_t length)
{
    if (!open_ || destination == nullptr)
    {
        return false;
    }

    if (static_cast<std::size_t>(start_address) + length >
        registers_.size())
    {
        return false;
    }

    for (std::size_t index = 0; index < length; ++index)
    {
        destination[index] =
            registers_[static_cast<std::size_t>(start_address) + index];
    }

    return true;
}

void SimulatedSpiBus::setRegister(
    const std::uint8_t register_address,
    const std::uint8_t value)
{
    registers_[register_address] = value;
}

}  // namespace avionics::platform