#include "stm32f4xx_hal.h"
#include "avionics/platform/stm32/Stm32Clock.hpp"

extern "C"
void SysTick_Handler(void)
{
    HAL_IncTick();
}

int main()
{
    HAL_Init();

    avionics::platform::Stm32Clock clock{};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpio{};
    gpio.Pin = GPIO_PIN_5;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(GPIOA, &gpio);
    
    while (true)
    {
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

	const auto start = clock.nowUs();

	while ((clock.nowUs() - start) < 500000ULL)
	{
	}
    }
}
