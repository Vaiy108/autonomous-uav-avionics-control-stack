#pragma once

#include <cstddef>
#include <cstdint>

namespace avionics::platform
{

class IUart
{
public:
    virtual ~IUart() = default;

    virtual bool open() = 0;
    virtual void close() = 0;

    [[nodiscard]] virtual bool isOpen() const = 0;

    virtual std::size_t read(
        std::uint8_t* destination,
        std::size_t maximum_length) = 0;

    virtual std::size_t write(
        const std::uint8_t* data,
        std::size_t length) = 0;
};

}  // namespace avionics::platform