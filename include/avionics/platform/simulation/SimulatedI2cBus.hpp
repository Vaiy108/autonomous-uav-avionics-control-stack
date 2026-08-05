#pragma once

#include "avionics/platform/interfaces/II2cBus.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace avionics::platform
{

class SimulatedI2cBus final : public II2cBus
{
public:
    bool open() override;
    void close() override;

    [[nodiscard]] bool isOpen() const override;

    bool writeRegister(
        std::uint8_t device_address,
        std::uint8_t register_address,
        std::uint8_t value) override;

    bool readRegister(
        std::uint8_t device_address,
        std::uint8_t register_address,
        std::uint8_t& value) override;

    bool readRegisters(
        std::uint8_t device_address,
        std::uint8_t start_address,
        std::uint8_t* destination,
        std::size_t length) override;

    void setRegister(
        std::uint8_t device_address,
        std::uint8_t register_address,
        std::uint8_t value);

private:
    static constexpr std::size_t DeviceCount{128};
    static constexpr std::size_t RegisterCount{256};

    bool open_{false};

    std::array<
        std::array<std::uint8_t, RegisterCount>,
        DeviceCount>
        registers_{};
};

}  // namespace avionics::platform