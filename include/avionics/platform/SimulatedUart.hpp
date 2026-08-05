#pragma once

#include "avionics/platform/IUart.hpp"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <string_view>

namespace avionics::platform
{

class SimulatedUart final : public IUart
{
public:
    bool open() override;
    void close() override;

    [[nodiscard]] bool isOpen() const override;

    std::size_t read(
        std::uint8_t* destination,
        std::size_t maximum_length) override;

    std::size_t write(
        const std::uint8_t* data,
        std::size_t length) override;

    void injectReceivedData(std::string_view data);

    [[nodiscard]] std::size_t available() const;

private:
    bool open_{false};
    std::deque<std::uint8_t> receive_buffer_{};
    std::deque<std::uint8_t> transmit_buffer_{};
};

}  // namespace avionics::platform