import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

CSV_PATH = Path("position_hold_6dof.csv")
OUTPUT_DIR = Path("analysis")
OUTPUT_DIR.mkdir(exist_ok=True)

RAD2DEG = 180.0 / np.pi


def settling_time(time, response, target, tolerance):
    error = np.abs(response - target)

    for i in range(len(time)):
        if np.all(error[i:] <= tolerance):
            return time[i]

    return np.nan


def main():
    data = np.genfromtxt(
        CSV_PATH,
        delimiter=",",
        names=True
    )

    t = data["time"]

    x_sp = data["x_setpoint"]
    y_sp = data["y_setpoint"]

    x = data["x"]
    y = data["y"]
    z = data["z"]

    roll_sp = data["roll_setpoint"] * RAD2DEG
    pitch_sp = data["pitch_setpoint"] * RAD2DEG

    roll = data["roll"] * RAD2DEG
    pitch = data["pitch"] * RAD2DEG

    altitude_sp = data["altitude_setpoint"]

    motors = np.vstack([
        data["fl"],
        data["fr"],
        data["rl"],
        data["rr"]
    ])

    horizontal_error = np.sqrt(
        (x_sp - x) ** 2 +
        (y_sp - y) ** 2
    )

    final_error = horizontal_error[-1]

    x_overshoot = max(
        0.0,
        (np.max(x) - x_sp[-1]) /
        abs(x_sp[-1]) * 100.0
    )

    y_overshoot = max(
        0.0,
        (np.max(y) - y_sp[-1]) /
        abs(y_sp[-1]) * 100.0
    )

    x_settle = settling_time(
        t, x, x_sp[-1], 0.02 * abs(x_sp[-1])
    )

    y_settle = settling_time(
        t, y, y_sp[-1], 0.02 * abs(y_sp[-1])
    )

    saturation_samples = np.sum(
        (motors <= 1e-6) |
        (motors >= 1.0 - 1e-6)
    )

    print("=== 6-DOF Position-Hold Validation ===")
    print(f"Target X/Y: {x_sp[-1]:.3f} / {y_sp[-1]:.3f} m")
    print(f"Final X/Y: {x[-1]:.3f} / {y[-1]:.3f} m")
    print(f"Final horizontal error: {final_error:.4f} m")
    print()
    print(f"X overshoot: {x_overshoot:.2f} %")
    print(f"Y overshoot: {y_overshoot:.2f} %")
    print(f"X settling time (2%): {x_settle:.2f} s")
    print(f"Y settling time (2%): {y_settle:.2f} s")
    print()
    print(f"Altitude range: {np.min(z):.4f} to {np.max(z):.4f} m")
    print(f"Final altitude: {z[-1]:.4f} m")
    print(f"Maximum roll: {np.max(np.abs(roll)):.2f} deg")
    print(f"Maximum pitch: {np.max(np.abs(pitch)):.2f} deg")
    print(f"Motor saturation samples: {saturation_samples}")

    # -------------------------------------------------
    # Plot 1: X/Y tracking
    # -------------------------------------------------
    plt.figure(figsize=(9, 5))

    plt.plot(t, x_sp, "--", label="X setpoint")
    plt.plot(t, x, label="X position")

    plt.plot(t, y_sp, "--", label="Y setpoint")
    plt.plot(t, y, label="Y position")

    plt.xlabel("Time [s]")
    plt.ylabel("Position [m]")
    plt.title("6-DOF Position Tracking")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    plt.savefig(
        OUTPUT_DIR / "position_tracking_6dof.png",
        dpi=180
    )

    plt.close()

    # -------------------------------------------------
    # Plot 2: XY trajectory
    # -------------------------------------------------
    plt.figure(figsize=(7, 6))

    plt.plot(x, y, label="Vehicle trajectory")

    plt.scatter(
        [x[0]],
        [y[0]],
        marker="o",
        label="Start"
    )

    plt.scatter(
        [x_sp[-1]],
        [y_sp[-1]],
        marker="x",
        s=80,
        label="Target"
    )

    plt.scatter(
        [x[-1]],
        [y[-1]],
        marker="s",
        label="Final position"
    )

    plt.xlabel("X position [m]")
    plt.ylabel("Y position [m]")
    plt.title("6-DOF XY Flight Trajectory")
    plt.grid(True)
    plt.axis("equal")
    plt.legend()
    plt.tight_layout()

    plt.savefig(
        OUTPUT_DIR / "xy_trajectory_6dof.png",
        dpi=180
    )

    plt.close()

    # -------------------------------------------------
    # Plot 3: attitude tracking
    # -------------------------------------------------
    plt.figure(figsize=(9, 5))

    plt.plot(
        t,
        roll_sp,
        "--",
        label="Roll setpoint"
    )

    plt.plot(
        t,
        roll,
        label="Roll response"
    )

    plt.plot(
        t,
        pitch_sp,
        "--",
        label="Pitch setpoint"
    )

    plt.plot(
        t,
        pitch,
        label="Pitch response"
    )

    plt.xlabel("Time [s]")
    plt.ylabel("Attitude [deg]")
    plt.title("6-DOF Attitude Tracking During Position Control")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    plt.savefig(
        OUTPUT_DIR / "attitude_tracking_6dof.png",
        dpi=180
    )

    plt.close()

    # -------------------------------------------------
    # Plot 4: altitude hold
    # -------------------------------------------------
    plt.figure(figsize=(9, 5))

    plt.plot(
        t,
        altitude_sp,
        "--",
        label="Altitude setpoint"
    )

    plt.plot(
        t,
        z,
        label="Altitude response"
    )

    plt.xlabel("Time [s]")
    plt.ylabel("Altitude [m]")
    plt.title("6-DOF Altitude Hold During Position Control")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    plt.savefig(
        OUTPUT_DIR / "altitude_hold_6dof.png",
        dpi=180
    )

    plt.close()

    print("\nPlots written to:")
    print(" analysis/position_tracking_6dof.png")
    print(" analysis/xy_trajectory_6dof.png")
    print(" analysis/attitude_tracking_6dof.png")
    print(" analysis/altitude_hold_6dof.png")


if __name__ == "__main__":
    main()
