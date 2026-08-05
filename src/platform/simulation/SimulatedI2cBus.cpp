#include "avionics/platform/simulation/SimulatedI2cBus.hpp"

namespace avionics::platform
{

bool SimulatedI2cBus::open()
{
    open_ = true;
    return true;
}

void SimulatedI2cBus::close()
{
    open_ = false;
}

bool SimulatedI2cBus::isOpen() const
{
    return open_;
}

bool SimulatedI2cBus::writeRegister(
    const std::uint8_t device_address,
    const std::uint8_t register_address,
    const std::uint8_t value)
{
    if (!open_ || device_address >= DeviceCount)
    {
        return false;
    }

    registers_[device_address][register_address] = value;
    return true;
}

bool SimulatedI2cBus::readRegister(
    const std::uint8_t device_address,
    const std::uint8_t register_address,
    std::uint8_t& value)
{
    if (!open_ || device_address >= DeviceCount)
    {
        return false;
    }

    value = registers_[device_address][register_address];
    return true;
}

bool SimulatedI2cBus::readRegisters(
    const std::uint8_t device_address,
    const std::uint8_t start_address,
    std::uint8_t* destination,
    const std::size_t length)
{
    if (!open_ ||
        destination == nullptr ||
        device_address >= DeviceCount)
    {
        return false;
    }

    if (static_cast<std::size_t>(start_address) + length >
        RegisterCount)
    {
        return false;
    }

    for (std::size_t index = 0; index < length; ++index)
    {
        destination[index] =
            registers_[device_address]
                      [static_cast<std::size_t>(start_address) + index];
    }

    return true;
}

void SimulatedI2cBus::setRegister(
    const std::uint8_t device_address,
    const std::uint8_t register_address,
    const std::uint8_t value)
{
    if (device_address < DeviceCount)
    {
        registers_[device_address][register_address] = value;
    }
}

}  // namespace avionics::platform