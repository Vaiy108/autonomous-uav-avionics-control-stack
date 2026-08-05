#pragma once

#include "avionics/platform/interfaces/ISpiBus.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace avionics::platform
{

class SimulatedSpiBus final : public ISpiBus
{
public:
    bool open() override;
    void close() override;

    [[nodiscard]] bool isOpen() const override;

    bool writeRegister(
        std::uint8_t register_address,
        std::uint8_t value) override;

    bool readRegister(
        std::uint8_t register_address,
        std::uint8_t& value) override;

    bool readRegisters(
        std::uint8_t start_address,
        std::uint8_t* destination,
        std::size_t length) override;

    void setRegister(
        std::uint8_t register_address,
        std::uint8_t value);

private:
    bool open_{false};
    std::array<std::uint8_t, 256> registers_{};
};

}  // namespace avionics::platform