#include "avionics/drivers/SimulatedGnssDriver.hpp"
#include "avionics/drivers/SimulatedImuDriver.hpp"
#include "avionics/middleware/LocalMessageBus.hpp"

#include <iomanip>
#include <iostream>

int main()
{
    avionics::drivers::SimulatedImuDriver imu{};
    avionics::drivers::SimulatedGnssDriver gnss{};
    avionics::middleware::LocalMessageBus message_bus{};

    message_bus.subscribeImu(
        [](const avionics::messages::ImuSample& sample)
        {
            std::cout
                << std::fixed
                << std::setprecision(3)
                << "IMU  "
                << "| t = " << sample.timestamp_us << " us "
                << "| accel = ("
                << sample.acceleration_mps2.x << ", "
                << sample.acceleration_mps2.y << ", "
                << sample.acceleration_mps2.z << ") m/s^2 "
                << "| gyro = ("
                << sample.angular_velocity_rps.x << ", "
                << sample.angular_velocity_rps.y << ", "
                << sample.angular_velocity_rps.z << ") rad/s\n";
        });

    message_bus.subscribeGnss(
        [](const avionics::messages::GnssSample& sample)
        {
            std::cout
                << std::fixed
                << std::setprecision(6)
                << "GNSS "
                << "| t = " << sample.timestamp_us << " us "
                << "| latitude = " << sample.latitude_deg << " deg "
                << "| longitude = " << sample.longitude_deg << " deg "
                << std::setprecision(2)
                << "| altitude = " << sample.altitude_m << " m "
                << "| speed = " << sample.ground_speed_mps << " m/s\n";
        });

    std::cout << "Embedded Avionics Software Stack\n\n";

    if (!imu.initialize() || !imu.selfTest())
    {
        std::cerr << "IMU initialization or self-test failed.\n";
        return 1;
    }

    if (!gnss.initialize() || !gnss.selfTest())
    {
        std::cerr << "GNSS initialization or self-test failed.\n";
        return 1;
    }

    std::cout
        << "Publishing IMU and GNSS samples through middleware:\n\n";

    /*
     * The IMU runs at 100 Hz and GNSS at 10 Hz.
     * For this compact demonstration, publish ten IMU samples
     * for every one GNSS sample.
     */
    for (int cycle = 0; cycle < 5; ++cycle)
    {
        for (int imu_index = 0; imu_index < 10; ++imu_index)
        {
            const auto imu_sample = imu.read();

            if (!imu_sample.valid)
            {
                std::cerr << "Invalid IMU sample.\n";
                return 1;
            }

            message_bus.publish(imu_sample);
        }

        const auto gnss_sample = gnss.read();

        if (!gnss_sample.valid)
        {
            std::cerr << "Invalid GNSS sample.\n";
            return 1;
        }

        message_bus.publish(gnss_sample);

        std::cout << '\n';
    }

    return 0;
}