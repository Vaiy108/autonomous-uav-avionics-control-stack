#include "avionics/drivers/SimulatedGnssDriver.hpp"
#include "avionics/drivers/SimulatedImuDriver.hpp"
#include "avionics/middleware/LocalMessageBus.hpp"
#include "avionics/drivers/SimulatedBarometerDriver.hpp"
#include "avionics/drivers/SimulatedMagnetometerDriver.hpp"
#include "avionics/middleware/SensorManager.hpp"
#include "avionics/services/SensorHealthMonitor.hpp"
#include "avionics/platform/SimulationClock.hpp"

#include <iomanip>
#include <iostream>

int main()
{
    //Clock
    avionics::platform::SimulationClock simulation_clock{};

    //Drivers
    avionics::drivers::SimulatedImuDriver imu{};
    avionics::drivers::SimulatedGnssDriver gnss{simulation_clock};
    avionics::drivers::SimulatedBarometerDriver barometer{};
    avionics::drivers::SimulatedMagnetometerDriver magnetometer{};
    avionics::middleware::LocalMessageBus message_bus{};
    avionics::middleware::SensorManager sensor_manager{message_bus};

    sensor_manager.start();

    avionics::services::SensorHealthMonitor health_monitor{sensor_manager};
    


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
        << "Publishing sensors samples through middleware:\n\n";

    /*
     * The IMU runs at 100 Hz and GNSS at 10 Hz.
     * For this compact demonstration, publish ten IMU samples
     * for every one GNSS sample.
     */
    for (int cycle = 0; cycle < 6; ++cycle)
    {
        const bool gnss_dropout_active =
            cycle >= 1 && cycle <= 3;

        if (cycle == 1)
        {
            std::cout
                << "\n*** FAULT INJECTION: GNSS DATA DROPOUT ACTIVE ***\n";
        }
        else if (cycle == 4)
        {
            std::cout
                << "\n*** RECOVERY: GNSS DATA STREAM RESTORED ***\n";
        }

        for (int step = 1; step <= 20; ++step)
        {
            //IMU
            //advance simulation time before reading the IMU:
            simulation_clock.advanceUs(10'000);
            const auto imu_sample = imu.read();

            if (!imu_sample.valid)
            {
                std::cerr << "Invalid IMU sample.\n";
                return 1;
            }

            message_bus.publish(imu_sample);

            if (step % 2 == 0)
            {
                //magnetometer
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
                //barometer
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

            if (step % 10 == 0 && !gnss_dropout_active)
            {
                //GNSS
                const auto gnss_sample = gnss.read();

                if (!gnss_sample.valid)
                {
                    std::cerr << "Invalid GNSS sample.\n";
                    return 1;
                }

                message_bus.publish(gnss_sample);
            }
        }
        // Sensor Manager
        if (sensor_manager.allSensorsAvailable())
        {
            const auto& latest_imu =
                sensor_manager.latestImu();
            const auto& latest_gnss =
                sensor_manager.latestGnss();
            const auto& latest_barometer =
                sensor_manager.latestBarometer();
            const auto& latest_magnetometer =
                sensor_manager.latestMagnetometer();

            std::cout
                << "\nSENSOR MANAGER SYNCHRONIZED SENSOR STATE\n"
                << "  IMU timestamp:  "
                << latest_imu.timestamp_us << " us\n"
                << "  GNSS timestamp: "
                << latest_gnss.timestamp_us << " us\n"
                << "  BARO timestamp: "
                << latest_barometer.timestamp_us << " us\n"
                << "  MAG timestamp:  "
                << latest_magnetometer.timestamp_us << " us\n"
                << "  GNSS position:  "
                << std::fixed
                << std::setprecision(6)
                << latest_gnss.latitude_deg << ", "
                << latest_gnss.longitude_deg << "\n"
                << std::setprecision(2)
                << "  Barometric altitude: "
                << latest_barometer.altitude_m << " m\n"
                << "  All sensors available: YES\n";
        }
        else
        {
            
            std::cout
                << "\nSensor Manager is waiting for all sensors.\n";
        }

        // Health Monitor

        //Using common clock:
        const auto current_time_us = simulation_clock.nowUs();

        // const auto current_time_us =
        //     sensor_manager.latestImu().timestamp_us;

        std::cout
            << "\nDEBUG\n"
            << "Current IMU time : " << current_time_us << " us\n"
            << "Latest GNSS time : " << sensor_manager.latestGnss().timestamp_us << " us\n"
            << "GNSS age         : "
            << current_time_us - sensor_manager.latestGnss().timestamp_us
            << " us\n";

        const auto system_health =
            health_monitor.evaluate(current_time_us);

        std::cout
            << "\nSENSOR HEALTH MONITOR\n"
            << "  IMU:  "
            << avionics::services::SensorHealthMonitor::toString(
                   system_health.imu.state)
            << " | age = "
            << system_health.imu.sample_age_us
            << " us\n"

            << "  GNSS: "
            << avionics::services::SensorHealthMonitor::toString(
                   system_health.gnss.state)
            << " | age = "
            << system_health.gnss.sample_age_us
            << " us\n"

            << "  BARO: "
            << avionics::services::SensorHealthMonitor::toString(
                   system_health.barometer.state)
            << " | age = "
            << system_health.barometer.sample_age_us
            << " us\n"

            << "  MAG:  "
            << avionics::services::SensorHealthMonitor::toString(
                   system_health.magnetometer.state)
            << " | age = "
            << system_health.magnetometer.sample_age_us
            << " us\n"

            << "  Overall system health: "
            << (system_health.allHealthy()
                    ? "HEALTHY"
                    : "DEGRADED")
            << '\n';

        std::cout << '\n';
    }
    

    return 0;
}