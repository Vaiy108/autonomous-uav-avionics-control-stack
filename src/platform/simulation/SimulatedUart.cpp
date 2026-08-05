#include "avionics/platform/simulation/SimulatedUart.hpp"

#include <algorithm>

namespace avionics::platform
{

bool SimulatedUart::open()
{
    open_ = true;
    return true;
}

void SimulatedUart::close()
{
    open_ = false;
}

bool SimulatedUart::isOpen() const
{
    return open_;
}

std::size_t SimulatedUart::read(std::uint8_t* destination, const std::size_t maximum_length)
{
    if (!open_ || destination == nullptr)
    {
        return 0;
    }

    const auto bytes_to_read =
        std::min(maximum_length, receive_buffer_.size());

    for (std::size_t index = 0;
         index < bytes_to_read;
         ++index)
    {
        destination[index] = receive_buffer_.front();
        receive_buffer_.pop_front();
    }

    return bytes_to_read;
}

std::size_t SimulatedUart::write(
    const std::uint8_t* data,
    const std::size_t length)
{
    if (!open_ || data == nullptr)
    {
        return 0;
    }

    for (std::size_t index = 0; index < length; ++index)
    {
        transmit_buffer_.push_back(data[index]);
    }

    return length;
}

void SimulatedUart::injectReceivedData(
    const std::string_view data)
{
    for (const char character : data)
    {
        receive_buffer_.push_back(
            static_cast<std::uint8_t>(character));
    }
}

std::size_t SimulatedUart::available() const
{
    return receive_buffer_.size();
}

}  // namespace avionics::platform