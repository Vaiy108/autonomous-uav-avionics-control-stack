# Project-Level Hazard Analysis

## Purpose

This document provides a simplified hazard analysis for the Embedded Avionics
Software Stack.

The goal is to identify representative sensor and software failure conditions,
evaluate their potential effect on flight software, and define detection and
mitigation mechanisms implemented or planned within the project.

This document is intended as an engineering demonstration of functional-safety
reasoning. It does not claim compliance with a specific aircraft certification
or safety-assessment standard.

---

## Severity Classification

The following qualitative severity levels are used:

| Severity | Description |
|---|---|
| S1 — Minor | Limited operational impact; system remains fully controllable |
| S2 — Major | Reduced functionality or degraded navigation/control capability |
| S3 — Hazardous | Significant degradation that could affect safe vehicle operation |
| S4 — Critical | Potential loss of essential flight-control or navigation capability |

---

## Hazard Analysis

| Hazard ID | Hazard / Failure Condition | Possible Cause | Potential Effect | Severity | Detection | Mitigation / Response | Related Requirements |
|---|---|---|---|---|---|---|---|
| HAZ-001 | GNSS data becomes stale or unavailable | Communication dropout, receiver fault, driver failure | Navigation position may become outdated or unavailable | S2 | Timestamp-based freshness monitoring | Mark GNSS as `STALE`, propagate `DEGRADED` system state, allow estimator/application to reject stale data | SWR-009, SWR-010, SWR-011, SWR-012 |
| HAZ-002 | GNSS data resumes after temporary dropout | Intermittent communication fault | Navigation solution may remain degraded if recovery is not detected | S2 | New valid timestamped GNSS measurement | Automatically return GNSS and overall system health to `HEALTHY` after valid data resumes | SWR-013 |
| HAZ-003 | IMU data becomes stale | Sensor fault, bus failure, task scheduling issue | Attitude/state estimation may become unreliable | S3 | IMU sample-age monitoring | Report IMU as `STALE`; propagate degraded system status; prevent stale measurements from being treated as current | SWR-009, SWR-010, SWR-011 |
| HAZ-004 | Sensor timestamps are inconsistent across drivers | Independent clocks, rollover error, platform timing mismatch | Incorrect sensor synchronization and inconsistent state-estimation inputs | S3 | Common timing abstraction and timestamp inspection | Use shared `IClock` abstraction for sensor timestamps | SWR-005, SWR-014 |
| HAZ-005 | Communication interface failure prevents sensor updates | UART/SPI/I2C driver failure or hardware communication fault | Loss of one or more sensor measurements | S2–S3 | Missing data / stale measurement detection | Isolate platform-specific interfaces behind abstractions; health monitor identifies stale/unavailable sensor | SWR-009, SWR-010, SWR-015 |
| HAZ-006 | One sensor failure causes complete software-stack failure | Tight coupling between sensor drivers and application logic | Loss of multiple avionics functions due to single component fault | S3 | Architecture inspection and integration testing | Publish-subscribe middleware decouples producers and consumers; failed sensor does not directly terminate unrelated sensor paths | SWR-006, SWR-007 |
| HAZ-007 | Incorrect GNSS fix is treated as valid navigation data | Poor satellite geometry, insufficient fix quality | Navigation function may use unreliable position information | S2 | `sensor_gps.fix_type` monitoring in PX4 module | Report GNSS as `DEGRADED` when fix quality is below the accepted threshold | SWR-019 |
| HAZ-008 | Embedded firmware exceeds available flash memory | Additional modules, drivers, logging, or vehicle-specific software | Firmware cannot link or cannot be deployed | S2 | Linker memory-region check | Review board configuration and remove nonessential functionality; verify final memory utilization | SWR-020 |
| HAZ-009 | Platform-specific dependency prevents deployment to target hardware | Driver directly depends on Windows/Linux/STM32-specific API | Software cannot be reused across target platforms | S1–S2 | Cross-platform build and code inspection | Hardware access through `IClock`, `IUart`, `ISpiBus`, and `II2cBus` abstractions | SWR-014, SWR-015, SWR-016 |
| HAZ-010 | Sensor update rate differs significantly from expected rate | Scheduling delay, sensor fault, middleware congestion | Reduced estimator performance or delayed fault detection | S2 | Update-rate monitoring and timestamps | Monitor sensor update frequency and freshness; flag unhealthy conditions when timing requirements are violated | SWR-008, SWR-009, SWR-018 |

---

## Hazard Mitigation Architecture

The main mitigation strategy is based on layered isolation and continuous
sensor-health monitoring.

```text
Physical / Simulated Sensor
          |
          v
Sensor Driver
          |
          v
Platform Abstraction
          |
          v
Publish-Subscribe Middleware
          |
          v
Sensor Manager
          |
          v
Health Monitor
          |
          +----> HEALTHY
          |
          +----> STALE / DEGRADED
          |
          v
Navigation / Flight Application
```

The architecture separates sensor acquisition from application logic and
prevents individual sensor drivers from being tightly coupled to downstream
consumers.

---


## Implemented Safety Mechanisms

The current software implements the following mechanisms:

- Common timestamp source through the `IClock` abstraction
- Timestamp-based freshness monitoring
- Stale-data detection
- Sensor availability monitoring
- System-level degraded-state propagation
- Controlled GNSS fault injection
- Automatic recovery following restoration of valid GNSS measurements
- Publish-subscribe isolation between drivers and consumers
- Multi-rate sensor handling
- PX4 IMU message-age monitoring
- PX4 GNSS fix-quality monitoring
- Embedded firmware memory-utilization verification


---


## Hazard Verification Examples

### HAZ-001 — GNSS Data Loss

A GNSS communication dropout is deliberately injected during SIL execution.

Expected sequence:

```text
GNSS HEALTHY
    |
    v
Communication dropout
    |
    v
GNSS timestamp stops updating
    |
    v
Sample age exceeds threshold
    |
    v
GNSS STALE
    |
    v
System DEGRADED
```

This scenario is verified by `SIL-FI-001`.


---


### HAZ-002 — GNSS Recovery

Following the injected dropout, GNSS publication is restored.

Expected sequence:

```text
System DEGRADED
    |
    v
New valid GNSS measurement
    |
    v
GNSS timestamp refreshed
    |
    v
GNSS HEALTHY
    |
    v
System HEALTHY
```

The demonstrated software automatically performs this recovery without manual
reset.


---


### HAZ-008 — Firmware Memory Constraint

During NuttX/FMUV6X integration, adding the custom PX4 avionics-monitor module
caused the firmware to exceed the configured FLASH region.

Observed condition:

```text
FLASH overflowed by 544 bytes
```

The board configuration was reviewed and an unused Gimbal module was disabled.

The firmware then successfully linked with approximately:

```text
FLASH usage: 99.18%
```

This demonstrates detection and mitigation of a target-specific embedded
resource constraint.


---


## Residual Risks / Not Yet Verified

The following areas are outside the current software-only validation scope:

- Physical sensor electrical failures
- Bus-level corruption on real UART/SPI/I2C interfaces
- Power-supply failures
- Physical actuator failures
- NuttX runtime execution on FMUv6X hardware
- Hardware-in-the-loop validation
- Flight-test validation
- Formal safety classification or certification assessment

These items are candidates for future HIL and hardware-validation activities.


---


## Traceability

| Hazard | Primary Requirements | Verification |
|---|---|---|
| HAZ-001 | SWR-009, SWR-010, SWR-011, SWR-012 | SIL-FI-001 |
| HAZ-002 | SWR-013 | SIL-FI-001 |
| HAZ-003 | SWR-009, SWR-010, SWR-011 | Health-monitor SIL |
| HAZ-004 | SWR-005, SWR-014 | Source inspection + SIL |
| HAZ-005 | SWR-009, SWR-010, SWR-015 | SIL / future HIL |
| HAZ-006 | SWR-006, SWR-007 | Integration test |
| HAZ-007 | SWR-019 | PX4-INT-001 |
| HAZ-008 | SWR-020 | NUTTX-BUILD-001 |
| HAZ-009 | SWR-014, SWR-015, SWR-016 | Linux build/runtime verification |
| HAZ-010 | SWR-008, SWR-009, SWR-018 | SIL + PX4 runtime monitoring |


---


## Conclusion

The project demonstrates a basic safety-oriented development flow:

```text
Hazard Identification
        |
        v
Software Requirement
        |
        v
Detection / Mitigation
        |
        v
SIL or Build Verification
        |
        v
Recorded PASS / NOT TESTED Result
```

This provides traceability between representative avionics hazards, implemented
software mitigations, and verification evidence.
