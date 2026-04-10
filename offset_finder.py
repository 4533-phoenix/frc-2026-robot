

import math
from bisect import bisect_right
from wpilog import DataLogReader
import numpy as np
from scipy.optimize import least_squares

"""
===============================================================================
HOW TO CALIBRATE CAMERA OFFSETS WITH THIS SCRIPT
===============================================================================
1. PREPARE COPROCESSORS: Go into your vision coprocessor settings and TURN OFF 
   "Constrained Solve" or any setting that forces the robot to a flat Z/Roll/Pitch 
   (e.g., in PhotonVision disable "Z/Rot Constrained"). You need raw 3D poses.
2. PLACE ROBOT: Put the robot down on the field (disabled mode is fine, 
   AdvantageKit will record). Ensure the cameras can clearly see multiple AprilTags.
3. SPIN (STATIONARY): DO NOT PUSH OR DRIVE THE ROBOT ACROSS THE FLOOR. Manually 
   spin the robot in place 360 degrees, nice and smoothly, for about 5-10 seconds.
4. MOVE AND REPEAT: Pick the robot entirely up (this breaks the clustering logic 
   so the math knows it moved) and place it in a new spot on the field with tags 
   in view. Perform another 360-degree stationary spin.
5. DOWNLOAD LOG: Turn off the robot, grab the USB or download the latest .wpilog.
6. UPDATE PATH: Change the `LOG_PATH` variable below to point to your new log.
7. RUN SCRIPT: The output will yield `X/Y/Z` and `Roll/Pitch/Yaw` deltas. 
8. APPLY CORRECTIONS: MATHEMATICALLY ADD the resulting translations and rotations 
   to your existing Java `Transform3d` camera configurations. DO NOT REPLACE your 
   old numbers with these—these are the mathematical CORRECTIONS for your current 
   assumptions.
===============================================================================
"""

LOG_PATH = "logs/akit_26-04-10_00-05-29_sccmp.wpilog"

reader = DataLogReader(LOG_PATH)
entries = {}

current_frame_time = None
current_data = {}

gyro_times = []
gyro_yaw = []
gyro_pitch = []
gyro_roll = []

def interpolate_rotation(t):
    if not gyro_times:
        return 0.0, 0.0, 0.0

    i2 = bisect_right(gyro_times, t)
    if i2 == 0:
        return gyro_roll[0], gyro_pitch[0], gyro_yaw[0]
    if i2 >= len(gyro_times):
        return gyro_roll[-1], gyro_pitch[-1], gyro_yaw[-1]

    i1 = i2 - 1
    # Use 4 surrounding points: i0, i1, i2, i3
    i0 = max(0, i1 - 1)
    i3 = min(len(gyro_times) - 1, i2 + 1)
    
    t0, t1, t2, t3 = gyro_times[i0], gyro_times[i1], gyro_times[i2], gyro_times[i3]
    y0, y1, y2, y3 = gyro_yaw[i0], gyro_yaw[i1], gyro_yaw[i2], gyro_yaw[i3]
    p0, p1, p2, p3 = gyro_pitch[i0], gyro_pitch[i1], gyro_pitch[i2], gyro_pitch[i3]
    r0, r1, r2, r3 = gyro_roll[i0], gyro_roll[i1], gyro_roll[i2], gyro_roll[i3]

    def catmull_rom(tt, time0, time1, time2, time3, val0, val1, val2, val3):
        if time2 == time0:
            v1 = 0
        else:
            v1 = (val2 - val0) / (time2 - time0)
            
        if time3 == time1:
            v2 = 0
        else:
            v2 = (val3 - val1) / (time3 - time1)
            
        dt = time2 - time1
        if dt == 0:
            return val1
            
        s = (tt - time1) / dt
        
        h1 = 2*s**3 - 3*s**2 + 1
        h2 = -2*s**3 + 3*s**2
        h3 = s**3 - 2*s**2 + s
        h4 = s**3 - s**2
        
        return h1*val1 + h2*val2 + h3*dt*v1 + h4*dt*v2

    int_y = catmull_rom(t, t0, t1, t2, t3, y0, y1, y2, y3)
    int_p = catmull_rom(t, t0, t1, t2, t3, p0, p1, p2, p3)
    int_r = catmull_rom(t, t0, t1, t2, t3, r0, r1, r2, r3)

    return int_r, int_p, int_y

vision_frames = []

def process_frame(timestamp, data):
    required = ["/Vision/TagCounts", "/Vision/CameraIds", "/Vision/VisionPoses", "/Vision/Timestamps"]
    if not all(k in data for k in required):
        return
        
    tag_counts = data["/Vision/TagCounts"]
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
    if not entry_name or (not entry_name.startswith("/Vision/") and not entry_name.startswith("/Drive/Gyro/Odometry")):
        continue

    rec_time = record.getTimestamp()
    if current_frame_time is None:
        current_frame_time = rec_time
        
    if rec_time != current_frame_time:
        if current_data.get("/Vision/TagCounts"):
            process_frame(current_frame_time, current_data)
        
        # Pull gyro data
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
        elif "Positions" in entry_name or "Timestamps" in entry_name:
            current_data[entry_name] = record.getDoubleArray()
    except Exception:
        pass

if current_data and current_data.get("/Vision/TagCounts"):
    process_frame(current_frame_time, current_data)

# Sort the gyro arrays by time just in case, and unwrap angles
if gyro_times:
    combined = sorted(zip(gyro_times, gyro_yaw, gyro_pitch, gyro_roll))
    gyro_times = [c[0] for c in combined]
    
    def unwrap(angles):
        if not angles: return []
        unwrapped = [angles[0]]
        for i in range(1, len(angles)):
            diff = angles[i] - unwrapped[-1]
            diff = (diff + math.pi) % (2 * math.pi) - math.pi
            unwrapped.append(unwrapped[-1] + diff)
        return unwrapped

    gyro_yaw = unwrap([c[1] for c in combined])
    gyro_pitch = unwrap([c[2] for c in combined])
    gyro_roll = unwrap([c[3] for c in combined])

def quat_to_euler(w, x, y, z):
    # roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

camera_data = {}

for timestamp, data in vision_frames:
    tag_counts = data["/Vision/TagCounts"]
    camera_ids = data["/Vision/CameraIds"]
    poses = data["/Vision/VisionPoses"]
    timestamps = data["/Vision/Timestamps"]

    pose_size = 7 if len(poses) >= len(camera_ids) * 7 else 3
    
    for i in range(min(len(tag_counts), len(camera_ids), len(timestamps), len(poses)//pose_size)):
        if tag_counts[i] > 0:
            cam_id = camera_ids[i]
            ts = timestamps[i]
            
            if pose_size == 7:
                px = poses[i*7]
                py = poses[i*7+1]
                pz = poses[i*7+2]
                qw = poses[i*7+3]
                qx = poses[i*7+4]
                qy = poses[i*7+5]
                qz = poses[i*7+6]
                proll, ppitch, pyaw = quat_to_euler(qw, qx, qy, qz)
            else:
                px = poses[i*3]
                py = poses[i*3+1]
                pz = 0.0
                proll = 0.0
                ppitch = 0.0
                pyaw = poses[i*3+2]
            
            # Interpolate rotation
            interp_roll, interp_pitch, interp_yaw = interpolate_rotation(ts)
            
            if cam_id not in camera_data:
                camera_data[cam_id] = []
            camera_data[cam_id].append({
                "ts": ts,
                "v_x": px, "v_y": py, "v_z": pz,
                "v_roll": proll, "v_pitch": ppitch, "v_yaw": pyaw,
                "i_roll": interp_roll, "i_pitch": interp_pitch, "i_yaw": interp_yaw
            })

print("\n--- 6-DOF Camera Offset Optimization ---")
print("NOTE: For X/Y/Z translation calibration to work best, the log should contain")
print("the robot rotating (spinning in place) at a fixed location on the field.\n")

def wrap_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

for cam_id, obs in camera_data.items():
    print(f"\nEvaluating Camera {cam_id} ({len(obs)} observations)")

    def residuals(offsets):
        dx, dy, dz, dr, dp, dyaw = offsets
        corr_xs, corr_ys, corr_zs = [], [], []
        
        # 1. Apply translation offsets using the Gyro's 3D rotation matrix
        for o in obs:
            cr = math.cos(o["i_roll"]); sr = math.sin(o["i_roll"])
            cp = math.cos(o["i_pitch"]); sp = math.sin(o["i_pitch"])
            cy = math.cos(o["i_yaw"]); sy = math.sin(o["i_yaw"])
            
            R_x = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
            R_y = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            R_z = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
            R = R_z @ R_y @ R_x
            
            # Rotate the bot-relative camera offset error to field space
            field_off = R @ np.array([dx, dy, dz])
            
            corr_xs.append(o["v_x"] + field_off[0])
            corr_ys.append(o["v_y"] + field_off[1])
            corr_zs.append(o["v_z"] + field_off[2])
            
        # Instead of a global mean, we split the observations into continuous spatial/temporal clusters
        # so that if the robot drives across the field, each "stationary" cluster gets its own mean!
        res = []
        
        # Simple clustering: If time jumps by >0.5s or distance > 1.5m, it's a new cluster
        clusters = []
        current_cluster = []
        for i, o in enumerate(obs):
            if not current_cluster:
                current_cluster.append(i)
                continue
                
            prev_i = current_cluster[-1]
            dt = o["ts"] - obs[prev_i]["ts"]
            dx_dist = abs(corr_xs[i] - corr_xs[prev_i])
            dy_dist = abs(corr_ys[i] - corr_ys[prev_i])
            
            if dt > 0.5 or dx_dist > 1.5 or dy_dist > 1.5:
                clusters.append(current_cluster)
                current_cluster = [i]
            else:
                current_cluster.append(i)
                
        if current_cluster:
            clusters.append(current_cluster)
            
        for cluster_indices in clusters:
            c_mean_x = np.mean([corr_xs[i] for i in cluster_indices])
            c_mean_y = np.mean([corr_ys[i] for i in cluster_indices])
            c_mean_z = np.mean([corr_zs[i] for i in cluster_indices])
            
            for i in cluster_indices:
                o = obs[i]
                # Translation residuals (minimize positional variance PER cluster)
                e_x = corr_xs[i] - c_mean_x
                e_y = corr_ys[i] - c_mean_y
                e_z = corr_zs[i] - c_mean_z
                
                # Rotation residuals (minimize angular error vs gyro)
                e_r = wrap_angle((o["v_roll"] + dr) - o["i_roll"])
                e_p = wrap_angle((o["v_pitch"] + dp) - o["i_pitch"])
                e_yaw = wrap_angle((o["v_yaw"] + dyaw) - o["i_yaw"])
    
                res.extend([e_x, e_y, e_z, e_r, e_p, e_yaw])
            
        return res

    # Initial guess: [X_off, Y_off, Z_off, Roll_off, Pitch_off, Yaw_off]
    initial_guess = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # Robust least-squares with soft_l1 loss
    result = least_squares(residuals, initial_guess, loss='soft_l1', f_scale=0.1)

    if result.success:
        dx, dy, dz, dr, dp, dyaw = result.x
        
        print(f"  Optimized Translation Corrections (meters):")
        print(f"    X: {dx:+.4f} m")
        print(f"    Y: {dy:+.4f} m")
        print(f"    Z: {dz:+.4f} m")
        print()
        print(f"  Optimized Rotation Corrections (degrees):")
        print(f"    Roll:  {math.degrees(dr):+.2f}°")
        print(f"    Pitch: {math.degrees(dp):+.2f}°")
        print(f"    Yaw:   {math.degrees(dyaw):+.2f}°")
        print("\n  Add these values mathematically to your CameraConfig Transform3d.")
    else:
        print("  Optimization failed for this camera.")
