#pragma once

#include <cstddef>
#include <cstdint>

namespace avionics::drivers::gnss
{

struct GgaFix
{
    double latitude_deg{0.0};
    double longitude_deg{0.0};
    double altitude_m{0.0};

    std::uint8_t fix_quality{0};
    std::uint8_t satellites{0};

    bool valid{false};
};

class NmeaGgaParser
{
public:
    /*
     * Feed one character at a time from the GNSS UART.
     *
     * Returns true whenever a complete GGA sentence
     * has been received and processed.
     */
    bool ingest(std::uint8_t byte, GgaFix& fix);

private:
    static constexpr std::size_t kBufferSize = 128;

    char buffer_[kBufferSize]{};
    std::size_t index_{0};

    bool parseSentence(GgaFix& fix);

    static double nmeaCoordinateToDegrees(
        const char* coordinate,
        char hemisphere);
};

} // namespace avionics::drivers::gnss
