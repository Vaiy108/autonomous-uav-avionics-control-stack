#include "avionics/platform/stm32/Stm32Uart.hpp"

namespace avionics::platform
{

Stm32Uart::Stm32Uart(
    USART_TypeDef* instance,
    std::uint32_t baud_rate)
    : instance_(instance),
      baud_rate_(baud_rate)
{
}

bool Stm32Uart::open()
{
    GPIO_InitTypeDef gpio{};

    /*
     * USART2
     * PA2  -> TX
     * PA3  -> RX
     *
     * Used for ST-LINK Virtual COM / Ubuntu debug output.
     */
    if (instance_ == USART2)
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_USART2_CLK_ENABLE();

        gpio.Pin       = GPIO_PIN_2 | GPIO_PIN_3;
        gpio.Mode      = GPIO_MODE_AF_PP;
        gpio.Pull      = GPIO_PULLUP;
        gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = GPIO_AF7_USART2;

        HAL_GPIO_Init(GPIOA, &gpio);
    }

    /*
     * USART1
     * PA9  -> TX
     * PA10 -> RX
     *
     * Used for the external NEO-M8N GNSS receiver.
     */
    else if (instance_ == USART1)
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_USART1_CLK_ENABLE();

        gpio.Pin       = GPIO_PIN_9 | GPIO_PIN_10;
        gpio.Mode      = GPIO_MODE_AF_PP;
        gpio.Pull      = GPIO_PULLUP;
        gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = GPIO_AF7_USART1;

        HAL_GPIO_Init(GPIOA, &gpio);
    }

    else
    {
        return false;
    }

    uart_.Instance          = instance_;
    uart_.Init.BaudRate     = baud_rate_;
    uart_.Init.WordLength   = UART_WORDLENGTH_8B;
    uart_.Init.StopBits     = UART_STOPBITS_1;
    uart_.Init.Parity       = UART_PARITY_NONE;
    uart_.Init.Mode         = UART_MODE_TX_RX;
    uart_.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    uart_.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&uart_) != HAL_OK)
    {
        open_ = false;
        return false;
    }

    open_ = true;
    return true;
}


void Stm32Uart::close()
{
    if (open_)
    {
        HAL_UART_DeInit(&uart_);
        open_ = false;
    }
}

bool Stm32Uart::isOpen() const
{
    return open_;
}

std::size_t Stm32Uart::write(
    const std::uint8_t* data,
    std::size_t length)
{
    if (!open_ || data == nullptr || length == 0)
    {
        return 0;
    }

    const HAL_StatusTypeDef status =
        HAL_UART_Transmit(
            &uart_,
            const_cast<std::uint8_t*>(data),
            static_cast<std::uint16_t>(length),
            HAL_MAX_DELAY);

    return status == HAL_OK ? length : 0;
}

std::size_t Stm32Uart::read(
    std::uint8_t* destination,
    std::size_t maximum_length)
{
    if (!open_ || destination == nullptr || maximum_length == 0)
    {
        return 0;
    }

    /*
     * Short timeout keeps the abstraction responsive rather than
     * blocking indefinitely when no UART data is available.
     */
    const HAL_StatusTypeDef status =
        HAL_UART_Receive(
            &uart_,
            destination,
            static_cast<std::uint16_t>(maximum_length),
            10U);

    return status == HAL_OK ? maximum_length : 0;
}

} // namespace avionics::platform
