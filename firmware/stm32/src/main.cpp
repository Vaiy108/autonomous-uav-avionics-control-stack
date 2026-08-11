#include "stm32f4xx_hal.h"

#include "avionics/platform/stm32/Stm32Clock.hpp"
#include "avionics/platform/stm32/Stm32Uart.hpp"
#include "avionics/drivers/gnss/NmeaGgaParser.hpp"
#include "avionics/messages/SensorMessages.hpp"

#include <cstdint>
#include <cstring>
#include <cstdio>

extern "C" void SysTick_Handler(void)
{
    HAL_IncTick();
}

int main()
{
    HAL_Init();

    avionics::platform::Stm32Clock clock{};

    /*
     * USART2:
     * STM32 -> ST-LINK Virtual COM -> Ubuntu
     */
    avionics::platform::Stm32Uart debug_uart{
        USART2,
        115200U
    };

    /*
     * USART1:
     * NEO-M8N -> STM32
     */
    avionics::platform::Stm32Uart gnss_uart{
        USART1,
        9600U
    };

    if (!debug_uart.open())
    {
        while (true)
        {
        }
    }

    if (!gnss_uart.open())
    {
        const char error[] =
            "ERROR: GNSS UART initialization failed\r\n";

        debug_uart.write(
            reinterpret_cast<const std::uint8_t*>(error),
            std::strlen(error));

        while (true)
        {
        }
    }

    const char startup[] =
        "\r\nEmbedded Avionics STM32 GNSS Bridge\r\n"
        "USART1 GNSS input  : 9600 baud\r\n"
        "USART2 debug output: 115200 baud\r\n"
        "Waiting for NEO-M8N data...\r\n\r\n";

    debug_uart.write(
        reinterpret_cast<const std::uint8_t*>(startup),
        std::strlen(startup));
        
    avionics::drivers::gnss::NmeaGgaParser parser{};
    avionics::drivers::gnss::GgaFix fix{};

    std::uint8_t byte{};
    
	while (true)
	{
	    const std::size_t received =
		gnss_uart.read(&byte, 1);

	    if (received != 1)
	    {
		continue;
	    }

	    if (parser.ingest(byte, fix))
		{
			avionics::messages::GnssSample sample{};

			sample.timestamp_us = clock.nowUs();

			sample.latitude_deg  = fix.latitude_deg;
			sample.longitude_deg = fix.longitude_deg;
			sample.altitude_m    = fix.altitude_m;

			/*
			 * GGA does not contain ground speed.
			 * We will obtain this from RMC later if needed.
			 */
			sample.ground_speed_mps = 0.0F;
			sample.valid = fix.valid;
			
			const std::uint32_t timestamp_ms =
        		static_cast<std::uint32_t>(
            		sample.timestamp_us / 1000ULL);

			char output[200]{};

			if (sample.valid)
			{
				std::snprintf(
				    output,
				    sizeof(output),
				    "GNSS SAMPLE | t=%lu ms | VALID | sats=%u | "
				    "lat=%.6f | lon=%.6f | alt=%.2f m\r\n",
				    static_cast<unsigned long>(timestamp_ms),
				    static_cast<unsigned>(fix.satellites),
				    sample.latitude_deg,
				    sample.longitude_deg,
				    sample.altitude_m);
			}
			else
			{
				std::snprintf(
				    output,
				    sizeof(output),
				    "GNSS SAMPLE | t=%lu ms | INVALID | sats=%u\r\n",
				    static_cast<unsigned long>(timestamp_ms),
            		static_cast<unsigned>(fix.satellites));
			}

			debug_uart.write(
				reinterpret_cast<const std::uint8_t*>(output),
				std::strlen(output));
		}
	}
}
