#pragma once

#include <cstdint>

namespace avionics::messages
{

using TimestampUs = std::uint64_t;

struct Vector3f
{
    float x{};
    float y{};
    float z{};
};

struct ImuSample
{
    TimestampUs timestamp_us{};

    Vector3f acceleration_mps2{};
    Vector3f angular_velocity_rps{};

    float temperature_c{};
    bool valid{false};
};

struct GnssSample
{
    TimestampUs timestamp_us{};

    double latitude_deg{};
    double longitude_deg{};
    float altitude_m{};

    float ground_speed_mps{};
    bool valid{false};
};

struct BarometerSample
{
    TimestampUs timestamp_us{};

    float pressure_pa{};
    float temperature_c{};
    float altitude_m{};

    bool valid{false};
};

struct MagnetometerSample
{
    TimestampUs timestamp_us{};

    Vector3f magnetic_field_ut{};

    bool valid{false};
};


}  // namespace avionics::messages