#include "stm32f4xx_hal.h"

#include "avionics/platform/stm32/Stm32Clock.hpp"
#include "avionics/platform/stm32/Stm32Uart.hpp"

#include <cstdint>
#include <cstring>

extern "C" void SysTick_Handler(void)
{
    HAL_IncTick();
}

int main()
{
    HAL_Init();

    avionics::platform::Stm32Clock clock{};
    avionics::platform::Stm32Uart uart{USART2, 115200U};

    if (!uart.open())
    {
        while (true)
        {
        }
    }

    const char startup[] =
        "\r\nEmbedded Avionics STM32 Backend\r\n"
        "Stm32Uart abstraction validation: OK\r\n";

    uart.write(
        reinterpret_cast<const std::uint8_t*>(startup),
        std::strlen(startup));

    while (true)
    {
        const char message[] =
            "STM32 IUart backend alive\r\n";

        uart.write(
            reinterpret_cast<const std::uint8_t*>(message),
            std::strlen(message));

        const auto start = clock.nowUs();

        while ((clock.nowUs() - start) < 1000000ULL)
        {
        }
    }
}
