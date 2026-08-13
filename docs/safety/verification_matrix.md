# Software Verification Matrix

## Purpose

This document provides traceability between the software requirements defined
in `software_requirements.md` and the verification activities performed on the
Autonomous UAV Avionics & Flight Control Software Stack.

Verification is performed primarily through software-in-the-loop (SIL),
integration testing, cross-platform compilation, PX4 SITL execution, and
embedded-target cross-compilation.

## Verification Methods

| Method | Description |
|---|---|
| INS | Source-code inspection |
| UT | Unit-level verification |
| IT | Integration test |
| SIL | Software-in-the-loop test |
| BUILD | Compilation / cross-compilation verification |
| RUNTIME | Executed runtime validation |

## Requirement Verification Matrix

| Requirement | Verification Method | Test / Evidence | Expected Result | Status |
|---|---|---|---|---|
| SWR-001 | SIL / IT | Simulated IMU driver initialized, sampled, and published through middleware | Valid IMU samples are generated and received | PASS |
| SWR-002 | SIL / IT | Simulated GNSS driver initialized, sampled, and published through middleware | Valid GNSS samples are generated and received | PASS |
| SWR-003 | SIL / IT | Simulated barometer driver initialized and published | Valid pressure, temperature, and altitude measurements are received | PASS |
| SWR-004 | SIL / IT | Simulated magnetometer driver initialized and published | Valid magnetic-field measurements are received | PASS |
| SWR-005 | INS / SIL | Sensor drivers obtain timestamps through the shared clock abstraction | Every valid sample contains a common-platform timestamp | PASS |
| SWR-006 | IT | Sensor drivers publish through the local publish-subscribe message bus | Subscribers receive sensor data without direct driver dependency | PASS |
| SWR-007 | IT | Sensor Manager stores the latest IMU, GNSS, barometer, and magnetometer samples | Latest measurements are available through the Sensor Manager | PASS |
| SWR-008 | SIL | Sensors execute at different simulated update rates | Multi-rate sensor streams are handled without loss of system operation | PASS |
| SWR-009 | SIL | Sensor Health Monitor calculates sample age from timestamps | Sensor freshness is determined from sample age | PASS |
| SWR-010 | SIL | GNSS updates intentionally suppressed beyond freshness threshold | GNSS state changes from HEALTHY to STALE | PASS |
| SWR-011 | SIL | GNSS stale condition propagated to overall system health | Overall system health changes to DEGRADED | PASS |
| SWR-012 | SIL | Controlled GNSS communication dropout injected | Health monitor detects missing GNSS updates | PASS |
| SWR-013 | SIL | GNSS publishing restored following dropout | GNSS and overall system health automatically return to HEALTHY | PASS |
| SWR-014 | INS / SIL | Drivers use `IClock` rather than maintaining independent timing state | Sensor timing is obtained from a common platform abstraction | PASS |
| SWR-015 | INS | UART, SPI and I2C platform interfaces separate communication from sensor/application logic | Application code remains independent of platform-specific bus APIs | PASS |
| SWR-016 | BUILD / RUNTIME | Clean GCC/CMake build and execution on Ubuntu 22.04 | Portable stack builds and executes successfully on Linux | PASS |
| SWR-017 | PX4 RUNTIME | `avionics_sensor_monitor` subscribes to `vehicle_acceleration` and `sensor_gps` using uORB | PX4 module receives sensor data through uORB | PASS |
| SWR-018 | PX4 RUNTIME | IMU message timestamp age and monitor update rate calculated in PX4 SITL | Module reports IMU freshness, rate, and health status | PASS |
| SWR-019 | PX4 RUNTIME | GNSS `fix_type` and satellite information monitored through `sensor_gps` | GNSS state is reported as HEALTHY or DEGRADED | PASS |
| SWR-020 | BUILD | Custom PX4 module cross-compiled and linked into FMUv6X NuttX/STM32H7 firmware | `avionics_sensor_monitor_main` is present in final ARM ELF image | PASS |

## SIL Fault-Injection Test

### Test ID: SIL-FI-001 — GNSS Communication Dropout

**Related requirements:**  
SWR-009, SWR-010, SWR-011, SWR-012, SWR-013

### Initial State

- IMU: HEALTHY
- GNSS: HEALTHY
- Barometer: HEALTHY
- Magnetometer: HEALTHY
- Overall System: HEALTHY

### Fault Injection

GNSS publication is intentionally disabled for multiple simulation cycles.

### Expected Behaviour

1. The last GNSS measurement remains available temporarily.
2. GNSS sample age increases.
3. When the configured freshness threshold is exceeded, GNSS transitions to `STALE`.
4. Overall system health transitions to `DEGRADED`.
5. Other sensors remain operational.
6. When GNSS publication resumes, a new timestamped measurement is received.
7. GNSS returns to `HEALTHY`.
8. Overall system health returns to `HEALTHY`.

### Result

**PASS**

Observed system transition:

```text
HEALTHY
   ↓
GNSS communication dropout
   ↓
GNSS sample age exceeds threshold
   ↓
GNSS STALE
   ↓
Overall system DEGRADED
   ↓
GNSS communication restored
   ↓
GNSS HEALTHY
   ↓
Overall system HEALTHY
```

## PX4 Integration Test

### Test ID: PX4-INT-001 — Multi-Sensor uORB Monitoring

**Related requirements:**  
SWR-017, SWR-018, SWR-019

### Test Environment

- Ubuntu 22.04
- PX4 SITL
- Gazebo X500
- uORB middleware
- Custom `avionics_sensor_monitor` module

### Verification

The module subscribes to:

```text
vehicle_acceleration
sensor_gps
```

and reports:

- IMU message age
- monitor update rate
- acceleration
- IMU health
- GNSS fix type
- satellite count
- GNSS health

### Result

**PASS**

The module executed successfully in PX4 SITL with no sensor timeout during the demonstrated run.

## NuttX Build Verification

### Test ID: NUTTX-BUILD-001 — FMUv6X Cross-Target Build

**Related requirement:**  
SWR-020

The custom PX4 module was enabled in the FMUv6X board configuration and
cross-compiled for the NuttX / STM32H7 target.

Verification of the final ARM ELF image confirmed the symbol:

```text
avionics_sensor_monitor_main
```

The resulting firmware package was successfully generated as:

```text
px4_fmu-v6x_default.px4
```

### Result

**PASS — Build and link validation**

Physical FMUv6X hardware execution has not yet been performed and is therefore
outside the scope of this verification result.

## Verification Summary

| Verification Area | Result |
|---|---|
| Sensor-driver integration | PASS |
| Publish-subscribe middleware | PASS |
| Multi-rate sensor handling | PASS |
| Sensor freshness monitoring | PASS |
| Fault detection | PASS |
| Degraded-state propagation | PASS |
| Automatic recovery | PASS |
| Linux portability | PASS |
| PX4/uORB integration | PASS |
| NuttX/STM32H7 build integration | PASS |
| Physical flight-controller execution | NOT TESTED |
| Hardware-in-the-loop validation | PLANNED |
