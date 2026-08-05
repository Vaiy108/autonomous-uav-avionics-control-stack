#pragma once

#include <cstddef>
#include <cstdint>

namespace avionics::platform
{

class ISpiBus
{
public:
    virtual ~ISpiBus() = default;

    virtual bool open() = 0;
    virtual void close() = 0;

    [[nodiscard]] virtual bool isOpen() const = 0;

    virtual bool writeRegister(
        std::uint8_t register_address,
        std::uint8_t value) = 0;

    virtual bool readRegister(
        std::uint8_t register_address,
        std::uint8_t& value) = 0;

    virtual bool readRegisters(
        std::uint8_t start_address,
        std::uint8_t* destination,
        std::size_t length) = 0;
};

}  // namespace avionics::platform