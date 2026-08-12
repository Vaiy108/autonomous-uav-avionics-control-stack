import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

CSV_PATH = Path("closed_loop_attitude.csv")


def settling_time(time, response, target, tolerance_fraction=0.02):
    tolerance = abs(target) * tolerance_fraction

    if tolerance == 0.0:
        tolerance = 1e-6

    error = np.abs(response - target)

    for i in range(len(time)):
        if np.all(error[i:] <= tolerance):
            return time[i]

    return np.nan


def rise_time(time, response, target):
    low = 0.10 * target
    high = 0.90 * target

    if target < 0:
        low, high = high, low

    low_idx = None
    high_idx = None

    for i, value in enumerate(response):
        if low_idx is None:
            if (target >= 0 and value >= low) or \
               (target < 0 and value <= low):
                low_idx = i

        if low_idx is not None:
            if (target >= 0 and value >= high) or \
               (target < 0 and value <= high):
                high_idx = i
                break

    if low_idx is None or high_idx is None:
        return np.nan

    return time[high_idx] - time[low_idx]


def percent_overshoot(response, target):
    if target == 0.0:
        return 0.0

    if target > 0:
        peak = np.max(response)
        return max(0.0, (peak - target) / abs(target) * 100.0)

    minimum = np.min(response)
    return max(0.0, (target - minimum) / abs(target) * 100.0)


def main():
    if not CSV_PATH.exists():
        raise FileNotFoundError(
            f"{CSV_PATH} not found. Run the closed-loop demo first."
        )

    data = np.genfromtxt(
        CSV_PATH,
        delimiter=",",
        names=True
    )

    time = data["time"]

    roll_setpoint = data["roll_setpoint"]
    roll = data["roll"]

    altitude_setpoint = data["altitude_setpoint"]
    altitude = data["altitude"]

    fl = data["fl"]
    fr = data["fr"]
    rl = data["rl"]
    rr = data["rr"]

    roll_target = roll_setpoint[-1]
    altitude_target = altitude_setpoint[-1]

    roll_rise = rise_time(time, roll, roll_target)

    roll_overshoot = percent_overshoot(
        roll,
        roll_target
    )

    roll_settle = settling_time(
        time,
        roll,
        roll_target,
        tolerance_fraction=0.02
    )

    roll_ss_error = abs(
        roll_target - roll[-1]
    )

    altitude_ss_error = abs(
        altitude_target - altitude[-1]
    )

    minimum_altitude = np.min(altitude)

    motors = np.vstack([fl, fr, rl, rr])

    saturation_events = np.sum(
        (motors <= 1e-6) |
        (motors >= 1.0 - 1e-6)
    )

    rad_to_deg = 180.0 / np.pi

    print("=== Closed-Loop Performance ===")
    print(
        f"Roll setpoint: "
        f"{roll_target * rad_to_deg:.3f} deg"
    )
    print(
        f"Final roll: "
        f"{roll[-1] * rad_to_deg:.3f} deg"
    )
    print(
        f"Roll steady-state error: "
        f"{roll_ss_error * rad_to_deg:.4f} deg"
    )
    print(
        f"Roll rise time (10-90%): "
        f"{roll_rise:.3f} s"
    )
    print(
        f"Roll overshoot: "
        f"{roll_overshoot:.2f} %"
    )
    print(
        f"Roll settling time (2%): "
        f"{roll_settle:.3f} s"
    )

    print()

    print(
        f"Altitude setpoint: "
        f"{altitude_target:.3f} m"
    )
    print(
        f"Final altitude: "
        f"{altitude[-1]:.3f} m"
    )
    print(
        f"Altitude steady-state error: "
        f"{altitude_ss_error:.4f} m"
    )
    print(
        f"Minimum altitude: "
        f"{minimum_altitude:.3f} m"
    )

    print()

    print(
        f"Motor saturation samples: "
        f"{saturation_events}"
    )

    # Roll response
    plt.figure(figsize=(8, 5))

    plt.plot(
        time,
        roll_setpoint * rad_to_deg,
        label="Roll Setpoint"
    )

    plt.plot(
        time,
        roll * rad_to_deg,
        label="Roll Response"
    )

    plt.xlabel("Time [s]")
    plt.ylabel("Roll Angle [deg]")
    plt.title("Closed-Loop Roll Response")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    plt.savefig(
        "analysis/roll_response.png",
        dpi=160
    )

    plt.close()

    # Altitude response
    plt.figure(figsize=(8, 5))

    plt.plot(
        time,
        altitude_setpoint,
        label="Altitude Setpoint"
    )

    plt.plot(
        time,
        altitude,
        label="Altitude Response"
    )

    plt.xlabel("Time [s]")
    plt.ylabel("Altitude [m]")
    plt.title("Closed-Loop Altitude Response")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    plt.savefig(
        "analysis/altitude_response.png",
        dpi=160
    )

    plt.close()

    print()
    print("Plots written to:")
    print("  analysis/roll_response.png")
    print("  analysis/altitude_response.png")


if __name__ == "__main__":
    main()
