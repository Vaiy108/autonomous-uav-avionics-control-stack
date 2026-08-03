#include "avionics/drivers/SimulatedGnssDriver.hpp"
#include "avionics/drivers/SimulatedImuDriver.hpp"
#include "avionics/middleware/LocalMessageBus.hpp"
#include "avionics/drivers/SimulatedBarometerDriver.hpp"
#include "avionics/drivers/SimulatedMagnetometerDriver.hpp"

#include <iomanip>
#include <iostream>

int main()
{
    avionics::drivers::SimulatedImuDriver imu{};
    avionics::drivers::SimulatedGnssDriver gnss{};
    avionics::drivers::SimulatedBarometerDriver barometer{};
    avionics::middleware::LocalMessageBus message_bus{};
    avionics::drivers::SimulatedMagnetometerDriver magnetometer{};


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

    message_bus.subscribeBarometer(
    [](const avionics::messages::BarometerSample& sample)
    {
        std::cout
            << std::fixed
            << std::setprecision(2)
            << "BARO "
            << "| t = " << sample.timestamp_us << " us "
            << "| pressure = " << sample.pressure_pa << " Pa "
            << "| temperature = " << sample.temperature_c << " C "
            << "| altitude = " << sample.altitude_m << " m\n";
    });

    message_bus.subscribeMagnetometer(
    [](const avionics::messages::MagnetometerSample& sample)
    {
        std::cout
            << std::fixed
            << std::setprecision(2)
            << "MAG  "
            << "| t = " << sample.timestamp_us << " us "
            << "| field = ("
            << sample.magnetic_field_ut.x << ", "
            << sample.magnetic_field_ut.y << ", "
            << sample.magnetic_field_ut.z << ") uT\n";
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

    if (!barometer.initialize() || !barometer.selfTest())
    {
        std::cerr << "Barometer initialization or self-test failed.\n";
        return 1;
    }

    if (!magnetometer.initialize() || !magnetometer.selfTest())
    {
        std::cerr
            << "Magnetometer initialization or self-test failed.\n";
        return 1;
    }

    std::cout
        << "Publishing IMU and GNSS samples through middleware:\n\n";

    /*
     * The IMU runs at 100 Hz and GNSS at 10 Hz.
     * For this compact demonstration, publish ten IMU samples
     * for every one GNSS sample.
     */
    for (int cycle = 0; cycle < 3; ++cycle)
    {
        for (int step = 1; step <= 20; ++step)
        {
            const auto imu_sample = imu.read();

            if (!imu_sample.valid)
            {
                std::cerr << "Invalid IMU sample.\n";
                return 1;
            }

            message_bus.publish(imu_sample);

            if (step % 2 == 0)
            {
                const auto magnetometer_sample =
                    magnetometer.read();

                if (!magnetometer_sample.valid)
                {
                    std::cerr
                        << "Invalid magnetometer sample.\n";
                    return 1;
                }

                message_bus.publish(magnetometer_sample);
            }

            if (step % 4 == 0)
            {
                const auto barometer_sample =
                    barometer.read();

                if (!barometer_sample.valid)
                {
                    std::cerr
                        << "Invalid barometer sample.\n";
                    return 1;
                }

                message_bus.publish(barometer_sample);
            }

            if (step % 10 == 0)
            {
                const auto gnss_sample = gnss.read();

                if (!gnss_sample.valid)
                {
                    std::cerr << "Invalid GNSS sample.\n";
                    return 1;
                }

                message_bus.publish(gnss_sample);
            }
        }

        std::cout << '\n';
    }
    

    return 0;
}