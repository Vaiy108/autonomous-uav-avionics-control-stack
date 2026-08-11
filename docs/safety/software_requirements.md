# Software Requirements

## Scope

This document defines software-level requirements for the Embedded Avionics
Sensor Platform and provides a basis for software-in-the-loop verification.

The requirements focus on sensor acquisition, timestamping, middleware,
sensor management, health monitoring, and fault handling.

## Functional Requirements

| ID | Requirement |
|---|---|
| SWR-001 | The software shall acquire IMU measurements through a sensor-driver interface. |
| SWR-002 | The software shall acquire GNSS measurements through a sensor-driver interface. |
| SWR-003 | The software shall acquire barometric measurements through a sensor-driver interface. |
| SWR-004 | The software shall acquire magnetometer measurements through a sensor-driver interface. |
| SWR-005 | Every valid sensor sample shall contain a timestamp supplied by the platform clock abstraction. |
| SWR-006 | Sensor drivers shall publish valid measurements through the middleware without direct dependency between publishers and consumers. |
| SWR-007 | The Sensor Manager shall maintain the latest available measurement from each monitored sensor. |
| SWR-008 | The software shall support sensor streams operating at different update rates. |

## Health-Monitoring Requirements

| ID | Requirement |
|---|---|
| SWR-009 | The software shall determine sensor freshness using the age of the latest received measurement. |
| SWR-010 | A sensor whose measurement exceeds its configured freshness threshold shall be reported as degraded or stale. |
| SWR-011 | The system health state shall indicate degraded operation when one or more required sensors are unhealthy. |
| SWR-012 | The health monitor shall detect loss of GNSS updates during an injected GNSS dropout. |
| SWR-013 | The health monitor shall return GNSS to a healthy state following restoration of valid GNSS measurements. |

## Platform Requirements

| ID | Requirement |
|---|---|
| SWR-014 | Hardware-dependent timing functionality shall be accessed through the `IClock` platform abstraction. |
| SWR-015 | Hardware communication interfaces shall be abstracted from application-level sensor processing. |
| SWR-016 | The portable software components shall compile and execute on the supported Linux/POSIX development environment. |

## PX4 Integration Requirements

| ID | Requirement |
|---|---|
| SWR-017 | The PX4 sensor-monitor module shall obtain sensor information through uORB. |
| SWR-018 | The PX4 module shall monitor IMU message freshness and update rate. |
| SWR-019 | The PX4 module shall report GNSS health using the available GNSS fix information. |
| SWR-020 | The PX4 module shall compile for a NuttX/STM32H7 PX4 target. |
