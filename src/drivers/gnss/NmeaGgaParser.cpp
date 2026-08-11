#include "avionics/drivers/gnss/NmeaGgaParser.hpp"

#include <cstdlib>
#include <cstring>

namespace avionics::drivers::gnss
{

bool NmeaGgaParser::ingest(
    std::uint8_t byte,
    GgaFix& fix)
{
    const char character =
        static_cast<char>(byte);

    /*
     * Start of a new NMEA sentence.
     */
    if (character == '$')
    {
        index_ = 0;
        buffer_[index_++] = character;
        return false;
    }

    if (index_ == 0)
    {
        return false;
    }

    /*
     * Protect against malformed/oversized sentences.
     */
    if (index_ >= kBufferSize - 1)
    {
        index_ = 0;
        return false;
    }

    /*
     * End of sentence.
     */
    if (character == '\n')
    {
        buffer_[index_] = '\0';

        const bool result =
            parseSentence(fix);

        index_ = 0;

        return result;
    }

    /*
     * Ignore carriage return.
     */
    if (character != '\r')
    {
        buffer_[index_++] = character;
    }

    return false;
}


bool NmeaGgaParser::parseSentence(
    GgaFix& fix)
{
    /*
     * Accept both multi-GNSS and GPS-only GGA messages.
     */
    if (std::strncmp(buffer_, "$GNGGA", 6) != 0 &&
        std::strncmp(buffer_, "$GPGGA", 6) != 0)
    {
        return false;
    }

    char* fields[16]{};
    std::size_t field_count = 0;

    fields[field_count++] = buffer_;

    /*
     * Split sentence in-place at commas and checksum marker.
     */
    for (std::size_t i = 0;
         buffer_[i] != '\0' &&
         field_count < 16;
         ++i)
    {
        if (buffer_[i] == ',' ||
            buffer_[i] == '*')
        {
            buffer_[i] = '\0';
            fields[field_count++] =
                &buffer_[i + 1];
        }
    }

    /*
     * GGA fields:
     *
     * 0  message type
     * 1  UTC time
     * 2  latitude
     * 3  N/S
     * 4  longitude
     * 5  E/W
     * 6  fix quality
     * 7  satellites used
     * 8  HDOP
     * 9  altitude
     */

    if (field_count < 10)
    {
        return false;
    }

    fix = {};

    fix.fix_quality =
        static_cast<std::uint8_t>(
            std::strtoul(fields[6], nullptr, 10));

    fix.satellites =
        static_cast<std::uint8_t>(
            std::strtoul(fields[7], nullptr, 10));

    /*
     * Fix quality 0 means no valid position.
     *
     * Still return true because a valid GGA sentence
     * was parsed; fix.valid describes position validity.
     */
    if (fix.fix_quality == 0 ||
        fields[2][0] == '\0' ||
        fields[4][0] == '\0')
    {
        fix.valid = false;
        return true;
    }

    fix.latitude_deg =
        nmeaCoordinateToDegrees(
            fields[2],
            fields[3][0]);

    fix.longitude_deg =
        nmeaCoordinateToDegrees(
            fields[4],
            fields[5][0]);

    fix.altitude_m =
        std::strtod(fields[9], nullptr);

    fix.valid = true;

    return true;
}


double NmeaGgaParser::nmeaCoordinateToDegrees(
    const char* coordinate,
    char hemisphere)
{
    const double raw =
        std::strtod(coordinate, nullptr);

    const int degrees =
        static_cast<int>(raw / 100.0);

    const double minutes =
        raw - static_cast<double>(degrees * 100);

    double decimal_degrees =
        static_cast<double>(degrees) +
        minutes / 60.0;

    if (hemisphere == 'S' ||
        hemisphere == 'W')
    {
        decimal_degrees =
            -decimal_degrees;
    }

    return decimal_degrees;
}

} // namespace avionics::drivers::gnss
