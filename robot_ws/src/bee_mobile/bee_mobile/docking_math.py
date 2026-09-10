# 
# Docking Math Utilities
#
# !! Work in progress !!, file used by the docking controller and docking test nodes.
#
# This module provides the kinematic and motor-command helpers used by the
# docking controller. It converts velocity targets into steering angles and
# wheel speeds, applies mechanical limits and friction compensation, manages
# pulse timing and filtering, and publishes commands to the ROS 2 wheel topics.
# 

import math
from std_msgs.msg import Float64MultiArray
from rclpy.time import Time
from rclpy.duration import Duration

def clamp_steering_angle(target_angle_deg, max_mechanical_angle_deg=90.0):
    """
    Clamps any steering angle that exceeds the physical limits of the servomotors.
    """
    return max(min(target_angle_deg, max_mechanical_angle_deg), -max_mechanical_angle_deg)

def normalize_steering_angle_and_speed(raw_angle_deg, logical_speed_ms):
    """
    Normalizes any wheel steering angle to the allowable +/- 90° range.
    If the raw angle is outside this range, flip the speed direction and add/subtract pi.
    """
    if raw_angle_deg > 90.0:
        return raw_angle_deg - 180.0, -logical_speed_ms
    elif raw_angle_deg < -90.0:
        return raw_angle_deg + 180.0, -logical_speed_ms
    return raw_angle_deg, logical_speed_ms

def apply_stiction_mapping(logical_speed_ms, wheel_name):
    """
    Asymmetric Coulomb friction model.
    Decouples the physical wheel position from its kinematic velocity vector.
    """
    V_MAX_MS = 0.67
    V_MIN_NAV2_MS = 0.10
    
    abs_val_ms = abs(logical_speed_ms)
    if abs_val_ms < 0.005: 
        return 0.0
    
    sign = math.copysign(1.0, logical_speed_ms)
        
    # Apply stiction according to the actual mechanical asymmetry.
    if 'left' in wheel_name:
        if sign > 0:
            # Robot moving forward: the left motor turns in the less efficient
            # "rear hardware" direction.
            v_min_motor_ms = 0.30  # Larger boost to compensate.
        else:
            # Robot moving backward: the left motor turns in the more efficient
            # "front hardware" direction.
            v_min_motor_ms = 0.30  # Less friction to overcome.
    else:
        # Right wheels (installed in the standard orientation).
        if sign > 0:
            # Robot moving forward: the right motor turns in the more efficient
            # "front hardware" direction.
            v_min_motor_ms = 0.30  # Standard reference value.
        else:
            # Robot moving backward: the right motor turns in the less efficient
            # "rear hardware" direction.
            v_min_motor_ms = 0.30  # Slight compensation required.
        
    m = (V_MAX_MS - v_min_motor_ms) / (V_MAX_MS - V_MIN_NAV2_MS)
    A = (2.0 * (v_min_motor_ms - m * V_MIN_NAV2_MS)) / math.sqrt(V_MIN_NAV2_MS)
    B = 2.0 * m - (v_min_motor_ms / V_MIN_NAV2_MS)
    
    if abs_val_ms <= V_MIN_NAV2_MS:
        mapped_ms = sign * (A * math.sqrt(abs_val_ms) + B * abs_val_ms)
    else:
        mapped_ms = sign * (m * (abs_val_ms - V_MIN_NAV2_MS) + v_min_motor_ms)
        
    return max(min(mapped_ms, V_MAX_MS), -V_MAX_MS)

def apply_steering_slew_rate(target_angle_deg, last_angle_deg, max_rate_deg_s=90.0, dt=0.05):
    """
    Limits the rotation speed of the steering modules.
    max_rate_deg_s: Maximum permitted angular speed (e.g. 180°/s).
    dt: Control-loop period (20 Hz = 0.05s).
    """
    max_step = max_rate_deg_s * dt
    error = target_angle_deg - last_angle_deg
    
    if abs(error) > max_step:
        return last_angle_deg + math.copysign(max_step, error)
    return target_angle_deg

def send_hardware_command(wheel_pubs, target_angles_deg, target_speeds_ms, wheel_radius_m, wheels_config, apply_stiction=True):
    """
    Applies stiction and clamping, converts kinematic commands to RPM and degrees,
    and publishes the final commands to the ROS 2 hardware topics.

    The published message is [rpm, steering_angle_deg].
    """
    
    for name, config in wheels_config.items():
        # Use a safe default if the wheel is missing from target_angles.
        raw_angle_deg = target_angles_deg.get(name, config.get('last_angle_deg', 0.0)) 
        cmd_angle_deg = clamp_steering_angle(raw_angle_deg)
        
        cmd_angle_deg = apply_steering_slew_rate(cmd_angle_deg, config.get('last_angle_deg', cmd_angle_deg))

        raw_speed_ms = target_speeds_ms.get(name, 0.0)

        # Conditional bypass.
        if apply_stiction:
            physical_speed_ms = apply_stiction_mapping(raw_speed_ms, name)
        else:
            physical_speed_ms = raw_speed_ms
        
        # Convert m/s to RPM for publication.
        rpm_final = (physical_speed_ms * 60.0) / (2.0 * math.pi * wheel_radius_m) * config['dir']
        
        config['last_angle_deg'] = cmd_angle_deg
        config['last_speed_ms'] = raw_speed_ms
        
        msg = Float64MultiArray(data=[float(rpm_final), float(cmd_angle_deg)])
        #print(name, msg)
        wheel_pubs[name].publish(msg)
        
def is_steering_aligned(wheels_config, target_angles_deg, tolerance_deg=5.0):
    """
    Simplified linear check. Circular handling and limit enforcement are delegated to the microcontroller firmware.
    """
    for name, config in wheels_config.items():
        target_ang_deg = clamp_steering_angle(target_angles_deg.get(name, 0.0))
        phys_angle_deg = config.get('current_angle_deg', 0.0)
        
        # Temporarily convert to radians for trigonometry only.
        delta_rad = math.radians(target_ang_deg - phys_angle_deg)
        delta_rad = math.atan2(math.sin(delta_rad), math.cos(delta_rad))
        delta_deg = math.degrees(delta_rad)
        
        if abs(delta_deg) > tolerance_deg:
            return False
            
    return True

def get_universal_transform(tf_buffer, parent_frame, child_frame, query_time=None):
    """
    Retrieves the global or historical transformation via TF2.
    Includes a non-blocking check to ensure frames exist before querying.
    """
    time_to_query = query_time if query_time else Time()
    
    # --- WAIT: Check whether the transform is physically available. ---
    # Do not call lookup_transform if the frames are not yet in the tree.
    if not tf_buffer.can_transform(parent_frame, child_frame, time_to_query, timeout=Duration(seconds=0.0)):
        return None
        
    try:
        trans = tf_buffer.lookup_transform(
            parent_frame, 
            child_frame, 
            time_to_query, 
            timeout=Duration(seconds=0.0) # Immediate because can_transform validated it.
        )
        t = trans.transform.translation
        r = trans.transform.rotation
        
        siny_cosp = 2.0 * (r.w * r.z + r.x * r.y)
        cosy_cosp = 1.0 - 2.0 * (r.y * r.y + r.z * r.z)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        
        return {
            'x_m': t.x, 'y_m': t.y, 'z_m': t.z,
            'qx': r.x, 'qy': r.y, 'qz': r.z, 'qw': r.w,
            'yaw_rad': yaw_rad,
            'stamp': trans.header.stamp
        }
    except Exception as e:
        # This should only occur for genuine temporal extrapolation errors.
        print(f"[TF2 EXTRAPOLATION DEBUG] : {str(e)}")
        return None

def send_pulse(state, elapsed_s, pulse_duration_s, target_vx_ms, target_vy_ms, target_wz_rad_s,
               filtered_vx_ms, filtered_vy_ms, filtered_wz_rad_s, alpha_v, alpha_wz,
               wheels_config, wheel_pubs, wheel_radius_m):

    """
    Evaluates the pulse-wait sequence with exponential filtering.
    Optimized to compute orientation and velocities based on exclusive active modes.
    """
    # Create targets using previous memory to prevent empty dicts between cycles
    target_angles_deg = {}

    for name, config in wheels_config.items():
        logical_speed_ms = 0.0
        #print(f"target_vx_ms={target_vx_ms:.1f}, target_vy_ms={target_vy_ms:.1f}, target_wz_rad_s={target_wz_rad_s:.1f}")

        if abs(target_wz_rad_s) > 0.0:
            # Temporary conversion after trigonometry.
            raw_angle_deg = math.degrees(math.atan2(config['x'], -config['y']))
            wheel_radius_from_center_m = math.hypot(config['x'], config['y'])
            logical_speed_ms = wheel_radius_from_center_m * target_wz_rad_s
        elif abs(target_vy_ms) > 0.0:
            if name in ['front_right', 'rear_left']:
                raw_angle_deg = 90.0
            else:
                raw_angle_deg = -90.0
        else:
            raw_angle_deg = 0.0

        target_angle_deg, logical_speed_ms = normalize_steering_angle_and_speed(raw_angle_deg, logical_speed_ms)
        target_angles_deg[name] = target_angle_deg
        #print(f"target_angle_deg={target_angle_deg:.1f}")
    
    zero_speeds_ms = {k: 0.0 for k in wheels_config.keys()}

    if state == 'WAITING':
        send_hardware_command(wheel_pubs, target_angles_deg, zero_speeds_ms, wheel_radius_m, wheels_config)
        state = 'WAITING_WHEEL_ORIENTATION'

    elif state == 'WAITING_WHEEL_ORIENTATION':
        send_hardware_command(wheel_pubs, target_angles_deg, zero_speeds_ms, wheel_radius_m, wheels_config)
        if is_steering_aligned(wheels_config, target_angles_deg):
            state = 'SENDING_PULSE'

    elif state == 'SENDING_PULSE':
        if elapsed_s < pulse_duration_s:
            filtered_vx_ms = (alpha_v * target_vx_ms) + ((1.0 - alpha_v) * filtered_vx_ms)
            filtered_vy_ms = (alpha_v * target_vy_ms) + ((1.0 - alpha_v) * filtered_vy_ms)
            filtered_wz_rad_s = (alpha_wz * target_wz_rad_s) + ((1.0 - alpha_wz) * filtered_wz_rad_s)

            target_speeds_ms = {}
            for name, config in wheels_config.items():
                if abs(target_wz_rad_s) > 0.0:
                    wheel_radius_from_center_m = math.hypot(config['x'], config['y'])
                    raw_speed = wheel_radius_from_center_m * filtered_wz_rad_s
                    # Temporary conversion after trigonometry.
                    raw_angle_deg = math.degrees(math.atan2(config['x'], -config['y']))
                    _, ideal_speed = normalize_steering_angle_and_speed(raw_angle_deg, raw_speed)
                    target_speeds_ms[name] = ideal_speed
                elif abs(target_vy_ms) > 0.0:
                    # Adapt the speed direction to the angle for a unified thrust vector.
                    target_speeds_ms[name] = filtered_vy_ms if target_angles_deg[name] > 0.0 else -filtered_vy_ms
                else:
                    target_speeds_ms[name] = filtered_vx_ms

            send_hardware_command(wheel_pubs, target_angles_deg, target_speeds_ms, wheel_radius_m, wheels_config, )

        else:
            send_hardware_command(wheel_pubs, target_angles_deg, zero_speeds_ms, wheel_radius_m, wheels_config)
            state = 'WAITING'

    return state, filtered_vx_ms, filtered_vy_ms, filtered_wz_rad_s

def send_pulse_smooth(state, elapsed_s, pulse_duration_s,
                       target_vx_ms, target_vy_ms, target_wz_rad_s,
                       filtered_vx_ms, filtered_vy_ms, filtered_wz_rad_s,
                       alpha_v, alpha_wz, mode, dt,
                       wheels_config, wheel_pubs, wheel_radius_m):
    """
    Same kinematic structure as send_pulse, but replaces the abrupt
    end-of-pulse cutoff with a smooth deceleration ramp.
      mode 1 : RC discharge
      mode 2 : exact mirror of the acceleration curve (zero jerk both ends)

    IMPORTANT: dt must be the REAL control loop period — not a guessed
    constant — otherwise the Vmax-at-T/2 guarantee silently breaks.
    """
    half_time_s = pulse_duration_s / 2.0
    half_steps = max(1.0, round(half_time_s / dt))

    # Amplification so the charging curve lands exactly on `target` at T/2
    amp_v  = 1.0 / (1.0 - math.pow(1.0 - alpha_v,  half_steps))
    amp_wz = 1.0 / (1.0 - math.pow(1.0 - alpha_wz, half_steps))

    # ── Angle targets — identical to send_pulse (geometry + fixed-diagonal
    #    crab trick, no unnecessary 180° reorientation on direction flip) ──
    target_angles_deg = {}
    for name, config in wheels_config.items():
        logical_speed_ms = 0.0
        if abs(target_wz_rad_s) > 0.0:
            raw_angle_deg = math.degrees(math.atan2(config['x'], -config['y']))
            r = math.hypot(config['x'], config['y'])
            logical_speed_ms = r * target_wz_rad_s
        elif abs(target_vy_ms) > 0.0:
            raw_angle_deg = 90.0 if name in ['front_right', 'rear_left'] else -90.0
        else:
            raw_angle_deg = 0.0

        target_angle_deg, _ = normalize_steering_angle_and_speed(raw_angle_deg, logical_speed_ms)
        target_angles_deg[name] = target_angle_deg

    zero_speeds_ms = {k: 0.0 for k in wheels_config.keys()}

    if state == 'WAITING':
        send_hardware_command(wheel_pubs, target_angles_deg, zero_speeds_ms, wheel_radius_m, wheels_config)
        state = 'WAITING_WHEEL_ORIENTATION'

    elif state == 'WAITING_WHEEL_ORIENTATION':
        send_hardware_command(wheel_pubs, target_angles_deg, zero_speeds_ms, wheel_radius_m, wheels_config)
        if is_steering_aligned(wheels_config, target_angles_deg):
            state = 'SENDING_PULSE'

    elif state == 'SENDING_PULSE':
        if elapsed_s <= half_time_s:
            # Phase 1 — acceleration (same for both modes)
            filtered_vx_ms    = alpha_v  * (target_vx_ms    * amp_v)  + (1.0 - alpha_v)  * filtered_vx_ms
            filtered_vy_ms    = alpha_v  * (target_vy_ms    * amp_v)  + (1.0 - alpha_v)  * filtered_vy_ms
            filtered_wz_rad_s = alpha_wz * (target_wz_rad_s * amp_wz) + (1.0 - alpha_wz) * filtered_wz_rad_s

        elif elapsed_s <= pulse_duration_s:
            if mode == 1:
                # Phase 2, mode 1 — RC discharge toward zero
                filtered_vx_ms    = (1.0 - alpha_v)  * filtered_vx_ms
                filtered_vy_ms    = (1.0 - alpha_v)  * filtered_vy_ms
                filtered_wz_rad_s = (1.0 - alpha_wz) * filtered_wz_rad_s
            else:
                # Phase 2, mode 2 — exact mirror of the charging curve.
                # Mirrored around half_steps (the true midpoint of the
                # charge/discharge curve), not around total_steps — this
                # matches the reference numpy script exactly and stays
                # correct even if pulse_duration_s / dt isn't a clean
                # multiple of 2 * half_steps.
                current_step = int(round(elapsed_s / dt))
                mirror_step  = max(0, (2 * half_steps) - current_step)

                decay_v  = 1.0 - math.pow(1.0 - alpha_v,  mirror_step)
                decay_wz = 1.0 - math.pow(1.0 - alpha_wz, mirror_step)

                filtered_vx_ms    = target_vx_ms    * amp_v  * decay_v
                filtered_vy_ms    = target_vy_ms    * amp_v  * decay_v
                filtered_wz_rad_s = target_wz_rad_s * amp_wz * decay_wz
        else:
            filtered_vx_ms = filtered_vy_ms = filtered_wz_rad_s = 0.0
            state = 'WAITING'

        # FIXED: this block now runs on EVERY cycle of SENDING_PULSE,
        # including phase 1. Previously it only ran in the `elif` (phase 2)
        # and final `else` branches — during phase 1, filtered_vx_ms was
        # updated in memory but never published. The robot kept receiving
        # the old WAITING_WHEEL_ORIENTATION command (speed = 0) for the
        # entire first half of the pulse, then suddenly received the
        # near-fully-charged value the instant phase 2 began. That silent
        # half-pulse plus sudden publish is exactly the jump you saw.
        target_speeds_ms = {}
        for name, config in wheels_config.items():
            if abs(target_wz_rad_s) > 0.0:
                r = math.hypot(config['x'], config['y'])
                raw_speed = r * filtered_wz_rad_s
                raw_angle_deg = math.degrees(math.atan2(config['x'], -config['y']))
                _, ideal_speed = normalize_steering_angle_and_speed(raw_angle_deg, raw_speed)
                target_speeds_ms[name] = ideal_speed
            elif abs(target_vy_ms) > 0.0:
                target_speeds_ms[name] = filtered_vy_ms if target_angles_deg[name] > 0.0 else -filtered_vy_ms
            else:
                target_speeds_ms[name] = filtered_vx_ms

        send_hardware_command(wheel_pubs, target_angles_deg, target_speeds_ms, wheel_radius_m, wheels_config, apply_stiction=False)

    return state, filtered_vx_ms, filtered_vy_ms, filtered_wz_rad_s