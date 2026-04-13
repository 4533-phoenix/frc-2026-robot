import math
from bisect import bisect_right
from typing import Dict, List, Optional, Tuple, Any
from wpilog import DataLogReader
import numpy as np
from scipy.optimize import least_squares

"""
===============================================================================
STOP-AND-GO CAMERA OFFSET CALIBRATOR
===============================================================================
1. Turn OFF Constrained Solve on the coprocessor.
2. Spin the robot in place, stopping completely for 2 seconds every ~30 degrees.
3. Move to a new spot and repeat.
4. Apply these offsets to Java, then turn Constrained Solve BACK ON.
===============================================================================
"""

LOG_PATH: str = "akit_26-04-10_12-34-28.wpilog"

reader: DataLogReader = DataLogReader(LOG_PATH)
entries: Dict[int, str] = {}

current_frame_time: Optional[float] = None
current_data: Dict[str, Any] = {}

gyro_times: List[float] = []
gyro_yaw: List[float] = []
gyro_pitch: List[float] = []
gyro_roll: List[float] = []


def interpolate_rotation(t: float) -> Tuple[float, float, float]:
    """
    Interpolates the robot's rotation (roll, pitch, yaw) at a given timestamp using Catmull-Rom splines.

    Args:
        t (float): The timestamp to interpolate at.

    Returns:
        Tuple[float, float, float]: The interpolated (roll, pitch, yaw) in radians.
    """
    if not gyro_times:
        return 0.0, 0.0, 0.0

    i2: int = bisect_right(gyro_times, t)
    if i2 == 0:
        return gyro_roll[0], gyro_pitch[0], gyro_yaw[0]
    if i2 >= len(gyro_times):
        return gyro_roll[-1], gyro_pitch[-1], gyro_yaw[-1]

    i1: int = i2 - 1
    i0: int = max(0, i1 - 1)
    i3: int = min(len(gyro_times) - 1, i2 + 1)

    t0, t1, t2, t3 = gyro_times[i0], gyro_times[i1], gyro_times[i2], gyro_times[i3]
    y0, y1, y2, y3 = gyro_yaw[i0], gyro_yaw[i1], gyro_yaw[i2], gyro_yaw[i3]
    p0, p1, p2, p3 = gyro_pitch[i0], gyro_pitch[i1], gyro_pitch[i2], gyro_pitch[i3]
    r0, r1, r2, r3 = gyro_roll[i0], gyro_roll[i1], gyro_roll[i2], gyro_roll[i3]

    def catmull_rom(
        tt: float,
        time0: float,
        time1: float,
        time2: float,
        time3: float,
        val0: float,
        val1: float,
        val2: float,
        val3: float,
    ) -> float:
        """
        Calculates the Catmull-Rom spline interpolation for a single value.
        """
        if time2 == time0:
            v1 = 0.0
        else:
            v1 = (val2 - val0) / (time2 - time0)

        if time3 == time1:
            v2 = 0.0
        else:
            v2 = (val3 - val1) / (time3 - time1)

        dt: float = time2 - time1
        if dt == 0:
            return val1

        s: float = (tt - time1) / dt
        h1: float = 2 * s**3 - 3 * s**2 + 1
        h2: float = -2 * s**3 + 3 * s**2
        h3: float = s**3 - 2 * s**2 + s
        h4: float = s**3 - s**2

        return h1 * val1 + h2 * val2 + h3 * dt * v1 + h4 * dt * v2

    int_y: float = catmull_rom(t, t0, t1, t2, t3, y0, y1, y2, y3)
    int_p: float = catmull_rom(t, t0, t1, t2, t3, p0, p1, p2, p3)
    int_r: float = catmull_rom(t, t0, t1, t2, t3, r0, r1, r2, r3)

    return int_r, int_p, int_y


vision_frames: List[Tuple[float, Dict[str, Any]]] = []


def process_frame(timestamp: float, data: Dict[str, Any]) -> None:
    """
    Processes a single vision frame from the log, extracting relevant tag data.

    Args:
        timestamp (float): The timestamp of the vision frame.
        data (Dict[str, Any]): The data dictionary containing vision information.
    """
    required: List[str] = [
        "/Vision/TagCounts",
        "/Vision/CameraIds",
        "/Vision/VisionPoses",
        "/Vision/Timestamps",
    ]
    if not all(k in data for k in required):
        return

    tag_counts: List[int] = data["/Vision/TagCounts"]
    if not any(t > 0 for t in tag_counts):
        return

    vision_frames.append((timestamp, dict(data)))


for record in reader:
    if record.isStart():
        data = record.getStartData()
        entries[data.entry] = data.name

    if record.isControl():
        continue

    entry_name = entries.get(record.getEntry())
    if not entry_name or (
        not entry_name.startswith("/Vision/")
        and not entry_name.startswith("/Drive/Gyro/Odometry")
    ):
        continue

    rec_time = record.getTimestamp()
    if current_frame_time is None:
        current_frame_time = rec_time

    if rec_time != current_frame_time:
        if current_data.get("/Vision/TagCounts"):
            process_frame(current_frame_time, current_data)

        if "/Drive/Gyro/OdometryYawTimestamps" in current_data:
            ts_array = current_data["/Drive/Gyro/OdometryYawTimestamps"]
            yaw_array = current_data.get("/Drive/Gyro/OdometryYawPositions", [])
            pitch_array = current_data.get("/Drive/Gyro/OdometryPitchPositions", [])
            roll_array = current_data.get("/Drive/Gyro/OdometryRollPositions", [])
            for i, ts in enumerate(ts_array):
                gyro_times.append(ts)
                gyro_yaw.append(yaw_array[i] if i < len(yaw_array) else 0.0)
                gyro_pitch.append(pitch_array[i] if i < len(pitch_array) else 0.0)
                gyro_roll.append(roll_array[i] if i < len(roll_array) else 0.0)

        current_frame_time = rec_time
        current_data = {}

    try:
        if entry_name in ["/Vision/TagCounts", "/Vision/CameraIds"]:
            current_data[entry_name] = record.getIntegerArray()
        elif (
            "Positions" in entry_name
            or "Timestamps" in entry_name
            or "Poses" in entry_name
        ):
            current_data[entry_name] = record.getDoubleArray()
    except Exception:
        pass

if current_data and current_data.get("/Vision/TagCounts"):
    process_frame(current_frame_time, current_data)

if gyro_times:
    combined: List[Tuple[float, float, float, float]] = sorted(
        zip(gyro_times, gyro_yaw, gyro_pitch, gyro_roll)
    )
    gyro_times = [c[0] for c in combined]

    def unwrap(angles: List[float]) -> List[float]:
        """
        Unwraps an array of angles to be continuous.
        """
        if not angles:
            return []
        unwrapped: List[float] = [angles[0]]
        for i in range(1, len(angles)):
            diff: float = angles[i] - unwrapped[-1]
            diff = (diff + math.pi) % (2 * math.pi) - math.pi
            unwrapped.append(unwrapped[-1] + diff)
        return unwrapped

    gyro_yaw = unwrap([c[1] for c in combined])
    gyro_pitch = unwrap([c[2] for c in combined])
    gyro_roll = unwrap([c[3] for c in combined])


def quat_to_euler(w: float, x: float, y: float, z: float) -> Tuple[float, float, float]:
    """
    Converts a quaternion into Euler angles (roll, pitch, yaw).
    """
    sinr_cosp: float = 2 * (w * x + y * z)
    cosr_cosp: float = 1 - 2 * (x * x + y * y)
    roll: float = math.atan2(sinr_cosp, cosr_cosp)

    sinp: float = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch: float = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp: float = 2 * (w * z + x * y)
    cosy_cosp: float = 1 - 2 * (y * y + z * z)
    yaw: float = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


camera_data: Dict[int, List[Dict[str, float]]] = {}

for timestamp, data in vision_frames:
    tag_counts = data["/Vision/TagCounts"]
    camera_ids = data["/Vision/CameraIds"]
    poses = data["/Vision/VisionPoses"]
    timestamps = data["/Vision/Timestamps"]

    pose_size = 7 if len(poses) >= len(camera_ids) * 7 else 3

    for i in range(
        min(len(tag_counts), len(camera_ids), len(timestamps), len(poses) // pose_size)
    ):
        if tag_counts[i] > 0:
            cam_id = camera_ids[i]
            ts = timestamps[i]

            if pose_size == 7:
                px, py, pz = poses[i * 7], poses[i * 7 + 1], poses[i * 7 + 2]
                qw, qx, qy, qz = (
                    poses[i * 7 + 3],
                    poses[i * 7 + 4],
                    poses[i * 7 + 5],
                    poses[i * 7 + 6],
                )
                proll, ppitch, pyaw = quat_to_euler(qw, qx, qy, qz)
            else:
                px, py, pz = poses[i * 3], poses[i * 3 + 1], 0.0
                proll, ppitch, pyaw = 0.0, 0.0, poses[i * 3 + 2]

            interp_roll, interp_pitch, interp_yaw = interpolate_rotation(ts)

            if cam_id not in camera_data:
                camera_data[cam_id] = []
            camera_data[cam_id].append(
                {
                    "ts": ts,
                    "v_x": px,
                    "v_y": py,
                    "v_z": pz,
                    "v_roll": proll,
                    "v_pitch": ppitch,
                    "v_yaw": pyaw,
                    "i_roll": interp_roll,
                    "i_pitch": interp_pitch,
                    "i_yaw": interp_yaw,
                }
            )

print("\n--- 6-DOF Camera Offset Optimization ---")


def wrap_angle(angle: float) -> float:
    """
    Wraps an angle to the range [-pi, pi].
    """
    return (angle + np.pi) % (2 * np.pi) - np.pi


for cam_id, obs in camera_data.items():
    print(f"\nEvaluating Camera {cam_id} ({len(obs)} observations)")

    def residuals(offsets: np.ndarray) -> List[float]:
        """
        Calculates the residuals for the least squares optimization.

        Args:
            offsets (np.ndarray): The current guess for [dx, dy, dz, dr, dp, dyaw].

        Returns:
            List[float]: The list of residuals for all observations.
        """
        dx, dy, dz, dr, dp, dyaw = offsets
        corr_xs, corr_ys, corr_zs = [], [], []

        for o in obs:
            cr = math.cos(o["i_roll"])
            sr = math.sin(o["i_roll"])
            cp = math.cos(o["i_pitch"])
            sp = math.sin(o["i_pitch"])
            cy = math.cos(o["i_yaw"])
            sy = math.sin(o["i_yaw"])

            R_x = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
            R_y = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            R_z = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
            R = R_z @ R_y @ R_x

            field_off = R @ np.array([dx, dy, dz])

            corr_xs.append(o["v_x"] + field_off[0])
            corr_ys.append(o["v_y"] + field_off[1])
            corr_zs.append(o["v_z"] + field_off[2])

        res: List[float] = []
        clusters: List[List[int]] = []
        current_cluster: List[int] = []
        for i, o in enumerate(obs):
            if not current_cluster:
                current_cluster.append(i)
                continue

            prev_i: int = current_cluster[-1]
            dt: float = o["ts"] - obs[prev_i]["ts"]
            dx_dist: float = abs(corr_xs[i] - corr_xs[prev_i])
            dy_dist: float = abs(corr_ys[i] - corr_ys[prev_i])

            if dt > 0.5 or dx_dist > 1.5 or dy_dist > 1.5:
                clusters.append(current_cluster)
                current_cluster = [i]
            else:
                current_cluster.append(i)

        if current_cluster:
            clusters.append(current_cluster)

        for cluster_indices in clusters:
            c_mean_x: float = float(np.median([corr_xs[i] for i in cluster_indices]))
            c_mean_y: float = float(np.median([corr_ys[i] for i in cluster_indices]))
            c_mean_z: float = float(np.median([corr_zs[i] for i in cluster_indices]))

            for i in cluster_indices:
                o = obs[i]
                e_x = corr_xs[i] - c_mean_x
                e_y = corr_ys[i] - c_mean_y
                e_z = corr_zs[i] - c_mean_z

                e_r = wrap_angle((o["v_roll"] + dr) - o["i_roll"])
                e_p = wrap_angle((o["v_pitch"] + dp) - o["i_pitch"])
                e_yaw = wrap_angle((o["v_yaw"] + dyaw) - o["i_yaw"])

                res.extend([e_x, e_y, e_z, e_r, e_p, e_yaw])

        return res

    initial_guess = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    result = least_squares(residuals, initial_guess, loss="huber", f_scale=0.05)

    if result.success:
        dx, dy, dz, dr, dp, dyaw = result.x

        print("  Optimized Translation Corrections (meters):")
        print(f"    X: {dx:+.4f} m")
        print(f"    Y: {dy:+.4f} m")
        print(f"    Z: {dz:+.4f} m")
        print()
        print("  Optimized Rotation Corrections (degrees):")
        print(f"    Roll:  {math.degrees(dr):+.2f}°")
        print(f"    Pitch: {math.degrees(dp):+.2f}°")
        print(f"    Yaw:   {math.degrees(dyaw):+.2f}°")
        print("\n  Add these values mathematically to your CameraConfig Transform3d.")
    else:
        print("  Optimization failed for this camera.")
