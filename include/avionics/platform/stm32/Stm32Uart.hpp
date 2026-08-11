#pragma once

#include "avionics/platform/interfaces/IUart.hpp"
#include "stm32f4xx_hal.h"

#include <cstddef>
#include <cstdint>

namespace avionics::platform
{

class Stm32Uart final : public IUart
{
public:
    explicit Stm32Uart(USART_TypeDef* instance = USART2,
                       std::uint32_t baud_rate = 115200U);

    bool open() override;
    void close() override;

    [[nodiscard]] bool isOpen() const override;

    std::size_t read(
        std::uint8_t* destination,
        std::size_t maximum_length) override;

    std::size_t write(
        const std::uint8_t* data,
        std::size_t length) override;

private:
    UART_HandleTypeDef uart_{};
    USART_TypeDef* instance_;
    std::uint32_t baud_rate_;
    bool open_{false};
};

} // namespace avionics::platform
