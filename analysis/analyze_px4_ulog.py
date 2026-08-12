from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from pyulog import ULog


LOG_PATH = Path(
    "results/px4/px4_sitl_autonomous_mission.ulg"
)

OUTPUT_DIR = Path("analysis/px4")
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

RAD2DEG = 180.0 / np.pi


def get_dataset(log, name):
    matches = [
        data for data in log.data_list
        if data.name == name
    ]

    if not matches:
        raise RuntimeError(
            f"Dataset '{name}' not found in ULog"
        )

    return matches[0].data


def time_seconds(timestamp):
    timestamp = np.asarray(timestamp, dtype=float)

    return timestamp * 1e-6
    

def interpolate(source_t, source_y, target_t):
    source_t = np.asarray(source_t)
    source_y = np.asarray(source_y)

    valid = (
        np.isfinite(source_t) &
        np.isfinite(source_y)
    )

    if np.count_nonzero(valid) < 2:
        return np.full_like(
            target_t,
            np.nan,
            dtype=float
        )

    return np.interp(
        target_t,
        source_t[valid],
        source_y[valid],
        left=np.nan,
        right=np.nan
    )

def quaternion_to_euler(qw, qx, qy, qz):
    # Roll
    sinr_cosp = 2.0 * (
        qw * qx + qy * qz
    )

    cosr_cosp = 1.0 - 2.0 * (
        qx * qx + qy * qy
    )

    roll = np.arctan2(
        sinr_cosp,
        cosr_cosp
    )

    # Pitch
    sinp = 2.0 * (
        qw * qy - qz * qx
    )

    sinp = np.clip(
        sinp,
        -1.0,
        1.0
    )

    pitch = np.arcsin(sinp)

    # Yaw
    siny_cosp = 2.0 * (
        qw * qz + qx * qy
    )

    cosy_cosp = 1.0 - 2.0 * (
        qy * qy + qz * qz
    )

    yaw = np.arctan2(
        siny_cosp,
        cosy_cosp
    )

    return roll, pitch, yaw


def extract_quaternion(data, prefix):
    """
    Supports pyulog field layouts such as:

        q[0], q[1], q[2], q[3]

    or:

        q_d[0], q_d[1], ...
    """

    keys = [
        f"{prefix}[0]",
        f"{prefix}[1]",
        f"{prefix}[2]",
        f"{prefix}[3]",
    ]

    for key in keys:
        if key not in data:
            raise RuntimeError(
                f"Quaternion field '{key}' not found"
            )

    return (
        np.asarray(data[keys[0]]),
        np.asarray(data[keys[1]]),
        np.asarray(data[keys[2]]),
        np.asarray(data[keys[3]]),
    )


def find_flight_interval(log):
    """
    Primary method:
        vehicle_status.arming_state == 2

    PX4 ARMING_STATE_ARMED is normally 2.

    Fallback:
        actuator motor command > 0.1
    """

    status = get_dataset(
        log,
        "vehicle_status"
    )

    status_t = time_seconds(
        status["timestamp"]
    )

    if "arming_state" in status:
        armed = (
            np.asarray(
                status["arming_state"]
            ) == 2
        )

        if np.any(armed):
            armed_indices = np.where(armed)[0]

            start = status_t[
                armed_indices[0]
            ]

            end = status_t[
                armed_indices[-1]
            ]

            print(
                "Flight interval detected "
                "from vehicle_status."
            )

            return start, end

    print(
        "Warning: armed state not detected. "
        "Using actuator activity fallback."
    )

    motors = get_dataset(
        log,
        "actuator_motors"
    )

    motor_t = time_seconds(
        motors["timestamp"]
    )

    channels = []

    for i in range(4):
        key = f"control[{i}]"

        if key in motors:
            channels.append(
                np.asarray(motors[key])
            )

    if not channels:
        raise RuntimeError(
            "Unable to detect flight interval"
        )

    motor_array = np.vstack(channels)

    active = np.any(
        motor_array > 0.1,
        axis=0
    )

    if not np.any(active):
        raise RuntimeError(
            "No active flight interval found"
        )

    indices = np.where(active)[0]

    return (
        motor_t[indices[0]],
        motor_t[indices[-1]]
    )


def trim(t, start, end, *signals):
    mask = (
        (t >= start) &
        (t <= end)
    )

    trimmed = [t[mask]]

    for signal in signals:
        trimmed.append(
            np.asarray(signal)[mask]
        )

    return trimmed


def rmse(error):
    error = np.asarray(error)

    valid = np.isfinite(error)

    if not np.any(valid):
        return np.nan

    return np.sqrt(
        np.mean(
            np.square(
                error[valid]
            )
        )
    )


def main():
    if not LOG_PATH.exists():
        raise FileNotFoundError(
            f"ULog not found: {LOG_PATH}"
        )

    print(
        f"Loading {LOG_PATH} ..."
    )

    log = ULog(str(LOG_PATH))

    flight_start, flight_end = (
        find_flight_interval(log)
    )

    duration = (
        flight_end -
        flight_start
    )

    print(
        f"Detected flight duration: "
        f"{duration:.2f} s"
    )

    # -------------------------------------------------
    # Local position
    # -------------------------------------------------

    position = get_dataset(
        log,
        "vehicle_local_position"
    )

    pos_t = time_seconds(
        position["timestamp"]
    )

    pos_t, x, y, z = trim(
        pos_t,
        flight_start,
        flight_end,
        position["x"],
        position["y"],
        position["z"],
    )

    pos_t -= flight_start

    # -------------------------------------------------
    # Local position setpoint
    # -------------------------------------------------

    pos_sp = get_dataset(
        log,
        "vehicle_local_position_setpoint"
    )

    pos_sp_t = time_seconds(
        pos_sp["timestamp"]
    )

    pos_sp_t, x_sp, y_sp, z_sp = trim(
        pos_sp_t,
        flight_start,
        flight_end,
        pos_sp["x"],
        pos_sp["y"],
        pos_sp["z"],
    )

    pos_sp_t -= flight_start

    # Interpolate setpoint onto state timestamps.
    x_sp_i = interpolate(
        pos_sp_t,
        x_sp,
        pos_t
    )

    y_sp_i = interpolate(
        pos_sp_t,
        y_sp,
        pos_t
    )

    z_sp_i = interpolate(
        pos_sp_t,
        z_sp,
        pos_t
    )

    # -------------------------------------------------
    # Vehicle attitude
    # -------------------------------------------------

    attitude = get_dataset(
        log,
        "vehicle_attitude"
    )

    att_t = time_seconds(
        attitude["timestamp"]
    )

    qw, qx, qy, qz = (
        extract_quaternion(
            attitude,
            "q"
        )
    )

    roll, pitch, yaw = (
        quaternion_to_euler(
            qw, qx, qy, qz
        )
    )

    att_t, roll, pitch, yaw = trim(
        att_t,
        flight_start,
        flight_end,
        roll,
        pitch,
        yaw,
    )

    att_t -= flight_start

    # -------------------------------------------------
    # Attitude setpoint
    # -------------------------------------------------

    attitude_sp = get_dataset(
        log,
        "vehicle_attitude_setpoint"
    )

    att_sp_t = time_seconds(
        attitude_sp["timestamp"]
    )

    try:
        qd_w, qd_x, qd_y, qd_z = (
            extract_quaternion(
                attitude_sp,
                "q_d"
            )
        )

        roll_sp, pitch_sp, yaw_sp = (
            quaternion_to_euler(
                qd_w,
                qd_x,
                qd_y,
                qd_z
            )
        )

    except RuntimeError:
        # Compatibility with older PX4 layouts.
        roll_sp = np.asarray(
            attitude_sp["roll_body"]
        )

        pitch_sp = np.asarray(
            attitude_sp["pitch_body"]
        )

        yaw_sp = np.asarray(
            attitude_sp["yaw_body"]
        )

    (
        att_sp_t,
        roll_sp,
        pitch_sp,
        yaw_sp,
    ) = trim(
        att_sp_t,
        flight_start,
        flight_end,
        roll_sp,
        pitch_sp,
        yaw_sp,
    )

    att_sp_t -= flight_start

    roll_sp_i = interpolate(
        att_sp_t,
        roll_sp,
        att_t
    )

    pitch_sp_i = interpolate(
        att_sp_t,
        pitch_sp,
        att_t
    )

    # -------------------------------------------------
    # Motors
    # -------------------------------------------------

    motors = get_dataset(
        log,
        "actuator_motors"
    )

    motor_t = time_seconds(
        motors["timestamp"]
    )

    motor_channels = []

    for i in range(4):
        key = f"control[{i}]"

        if key not in motors:
            raise RuntimeError(
                f"Motor channel {key} missing"
            )

        motor_channels.append(
            np.asarray(
                motors[key]
            )
        )

    (
        motor_t,
        motor_1,
        motor_2,
        motor_3,
        motor_4,
    ) = trim(
        motor_t,
        flight_start,
        flight_end,
        *motor_channels
    )

    motor_t -= flight_start

    # -------------------------------------------------
    # Metrics
    # -------------------------------------------------

    x_error = x - x_sp_i
    y_error = y - y_sp_i
    z_error = z - z_sp_i


    horizontal_error = np.full_like(
		x_error,
		np.nan,
		dtype=float
	)

    valid_xy = (
		np.isfinite(x_error) &
		np.isfinite(y_error)
	)

    horizontal_error[valid_xy] = np.sqrt(
		x_error[valid_xy]**2 +
		y_error[valid_xy]**2
	)

    roll_error = (
        roll -
        roll_sp_i
    ) * RAD2DEG

    pitch_error = (
        pitch -
        pitch_sp_i
    ) * RAD2DEG

    motor_matrix = np.vstack([
        motor_1,
        motor_2,
        motor_3,
        motor_4
    ])

    valid_motor_values = (
        motor_matrix[
            np.isfinite(
                motor_matrix
            )
        ]
    )

    motor_min = np.min(
        valid_motor_values
    )

    motor_max = np.max(
        valid_motor_values
    )

    saturation_samples = np.sum(
        valid_motor_values >= 0.999
    )

    print()
    print(
        "=== PX4 SITL Flight Validation ==="
    )

    print(
        f"Flight duration: "
        f"{duration:.2f} s"
    )

    print()

    print(
        f"Horizontal position RMSE: "
        f"{rmse(horizontal_error):.3f} m"
    )

    print(
        f"Vertical position RMSE: "
        f"{rmse(z_error):.3f} m"
    )

    print()

    print(
        f"Roll tracking RMSE: "
        f"{rmse(roll_error):.3f} deg"
    )

    print(
        f"Pitch tracking RMSE: "
        f"{rmse(pitch_error):.3f} deg"
    )

    print()

    print(
        f"Motor command range: "
        f"{motor_min:.3f} to "
        f"{motor_max:.3f}"
    )

    print(
        f"Motor saturation samples: "
        f"{saturation_samples}"
    )

    # -------------------------------------------------
    # Plot 1: XY position tracking
    # -------------------------------------------------

    plt.figure(figsize=(9, 5))

    plt.plot(
        pos_t,
        x_sp_i,
        "--",
        label="X setpoint"
    )

    plt.plot(
        pos_t,
        x,
        label="X position"
    )

    plt.plot(
        pos_t,
        y_sp_i,
        "--",
        label="Y setpoint"
    )

    plt.plot(
        pos_t,
        y,
        label="Y position"
    )

    plt.xlabel("Flight time [s]")
    plt.ylabel("Local position [m]")
    plt.title(
        "PX4 SITL Local Position Tracking"
    )

    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    plt.savefig(
        OUTPUT_DIR /
        "px4_position_tracking.png",
        dpi=180
    )

    plt.close()

    # -------------------------------------------------
    # Plot 2: altitude tracking
    #
    # PX4 local Z uses NED:
    # negative Z = positive altitude.
    # Display altitude as -Z for readability.
    # -------------------------------------------------

    plt.figure(figsize=(9, 5))

    plt.plot(
        pos_t,
        -z_sp_i,
        "--",
        label="Altitude setpoint"
    )

    plt.plot(
        pos_t,
        -z,
        label="Estimated altitude"
    )

    plt.xlabel("Flight time [s]")
    plt.ylabel(
        "Altitude above local origin [m]"
    )

    plt.title(
        "PX4 SITL Altitude Tracking"
    )

    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    plt.savefig(
        OUTPUT_DIR /
        "px4_altitude_tracking.png",
        dpi=180
    )

    plt.close()

    # -------------------------------------------------
    # Plot 3: attitude tracking
    # -------------------------------------------------

    plt.figure(figsize=(9, 5))

    plt.plot(
        att_t,
        roll_sp_i * RAD2DEG,
        "--",
        label="Roll setpoint"
    )

    plt.plot(
        att_t,
        roll * RAD2DEG,
        label="Roll"
    )

    plt.plot(
        att_t,
        pitch_sp_i * RAD2DEG,
        "--",
        label="Pitch setpoint"
    )

    plt.plot(
        att_t,
        pitch * RAD2DEG,
        label="Pitch"
    )

    plt.xlabel("Flight time [s]")
    plt.ylabel("Attitude [deg]")

    plt.title(
        "PX4 SITL Attitude Tracking"
    )

    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    plt.savefig(
        OUTPUT_DIR /
        "px4_attitude_tracking.png",
        dpi=180
    )

    plt.close()

    # -------------------------------------------------
    # Plot 4: motor outputs
    # -------------------------------------------------

    plt.figure(figsize=(9, 5))

    plt.plot(
        motor_t,
        motor_1,
        label="Motor 1"
    )

    plt.plot(
        motor_t,
        motor_2,
        label="Motor 2"
    )

    plt.plot(
        motor_t,
        motor_3,
        label="Motor 3"
    )

    plt.plot(
        motor_t,
        motor_4,
        label="Motor 4"
    )

    plt.xlabel("Flight time [s]")
    plt.ylabel(
        "Normalized actuator command"
    )

    plt.title(
        "PX4 SITL Motor Commands"
    )

    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    plt.savefig(
        OUTPUT_DIR /
        "px4_motor_commands.png",
        dpi=180
    )

    plt.close()

    print()
    print("Plots written to:")

    for filename in [
        "px4_position_tracking.png",
        "px4_altitude_tracking.png",
        "px4_attitude_tracking.png",
        "px4_motor_commands.png",
    ]:
        print(
            f"  {OUTPUT_DIR / filename}"
        )


if __name__ == "__main__":
    main()
