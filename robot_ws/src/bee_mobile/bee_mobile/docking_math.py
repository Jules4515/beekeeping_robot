import math
from std_msgs.msg import Float64MultiArray
from rclpy.time import Time
from rclpy.duration import Duration

def clamp_steering_angle(target_angle_rad, max_mechanical_angle_rad=math.radians(90.0)):
    """
    Clamps any steering angle that exceeds the physical limits of the servomotors.
    """
    return max(min(target_angle_rad, max_mechanical_angle_rad), -max_mechanical_angle_rad)

def apply_stiction_mapping(logical_speed_ms):
    """
    Asymmetrical Coulomb friction model using a square root profile.
    Compensates for startup friction depending on the direction of movement.
    """
    V_MAX_MS = 0.67
    V_MIN_NAV2_MS = 0.10
    V_MIN_RIGHT_MS = 0.225
    V_MIN_LEFT_MS = 0.260
    
    abs_val_ms = abs(logical_speed_ms)
    if abs_val_ms < 0.005: 
        return 0.0
    
    sign = math.copysign(1.0, logical_speed_ms)
    v_min_motor_ms = V_MIN_LEFT_MS if sign > 0 else V_MIN_RIGHT_MS
    
    m = (V_MAX_MS - v_min_motor_ms) / (V_MAX_MS - V_MIN_NAV2_MS)
    A = (2.0 * (v_min_motor_ms - m * V_MIN_NAV2_MS)) / math.sqrt(V_MIN_NAV2_MS)
    B = 2.0 * m - (v_min_motor_ms / V_MIN_NAV2_MS)
    
    if abs_val_ms <= V_MIN_NAV2_MS:
        mapped_ms = sign * (A * math.sqrt(abs_val_ms) + B * abs_val_ms)
    else:
        mapped_ms = sign * (m * (abs_val_ms - V_MIN_NAV2_MS) + v_min_motor_ms)
        
    return max(min(mapped_ms, V_MAX_MS), -V_MAX_MS)

def send_hardware_command(wheel_pubs, target_angles_rad, target_speeds_ms, wheel_radius_m, wheels_config):
    """
    Applies stiction and clamping, converts kinematic commands to RPM and degrees,
    and publishes the final commands to the ROS 2 hardware topics.
    """
    for name, config in wheels_config.items():
        # .get() : sécurité si la clé name n'existe pas dans le dictionnaire target_angles met 0.0 en vitesse
        raw_angle_rad = target_angles_rad.get(name, config.get('last_angle_rad', 0.0)) 
        cmd_angle_rad = clamp_steering_angle(raw_angle_rad)
        
        raw_speed_ms = target_speeds_ms.get(name, 0.0)
        physical_speed_ms = apply_stiction_mapping(raw_speed_ms)
        
        rpm_final = (physical_speed_ms * 60.0) / (2.0 * math.pi * wheel_radius_m) * config['dir']
        cmd_angle_deg = math.degrees(cmd_angle_rad)
        
        config['last_angle_rad'] = cmd_angle_rad
        config['last_logical_speed_ms'] = raw_speed_ms
        
        msg = Float64MultiArray(data=[float(rpm_final), float(cmd_angle_deg)])
        wheel_pubs[name].publish(msg)

def is_steering_aligned(wheels_config, target_angles_rad, tolerance_rad=math.radians(5.0)):
    """
    Checks if all wheels are within the tolerance interval of their target angle.
    Uses atan2 to directly compute the shortest path difference.
    """
    for name, config in wheels_config.items():
        target_ang_rad = target_angles_rad.get(name, 0.0)
        # Assuming the hardware encoder callback stores the current angle in degrees
        phys_angle_rad = math.radians(config.get('current_angle_deg', 0.0))
        
        diff_rad = target_ang_rad - phys_angle_rad
        err_ang_rad = math.atan2(math.sin(diff_rad), math.cos(diff_rad))
        
        if abs(err_ang_rad) > tolerance_rad:
            return False
            
    return True

def get_universal_transform(tf_buffer, parent_frame, child_frame, query_time=None):
    """
    Retrieves the global or historical transformation via TF2.
    Returns translations in meters and rotations in radians.
    """
    time_to_query = query_time if query_time else Time()
    try:
        trans = tf_buffer.lookup_transform(parent_frame, child_frame, time_to_query, timeout=Duration(seconds=0.1))
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
    except Exception:
        return None

def send_pulse(state, elapsed_s, pulse_duration_s, target_vx_ms, target_vy_ms, target_wz_rad_s,
               filtered_vx_ms, filtered_vy_ms, filtered_wz_rad_s, alpha_v, alpha_wz,
               wheels_config, wheel_pubs, wheel_radius_m):
    """
    Evaluates the pulse-wait sequence with exponential filtering.
    Optimized to compute orientation and velocities based on exclusive active modes.
    """
    # Create targets using previous memory to prevent empty dicts between cycles
    target_angles_rad = {}

    for name, config in wheels_config.items():
        if abs(target_wz_rad_s) > 0.0:
            target_angles_rad[name] = math.atan2(config['x'], -config['y'])
        elif abs(target_vy_ms) > 0.0:
            target_angles_rad[name] = math.radians(60.0) if target_vy_ms > 0 else math.radians(-60.0)
        else:
            target_angles_rad[name] = 0.0
    
    zero_speeds_ms = {k: 0.0 for k in wheels_config.keys()}

    # 1. State: WAITING - command servos to rotate (traction remains at 0)
    if state == 'WAITING':
        send_hardware_command(wheel_pubs, target_angles_rad, zero_speeds_ms, wheel_radius_m, wheels_config)
        state = 'WAITING_WHEEL_ORIENTATION'

    # 2. State: WAITING_WHEEL_ORIENTATION - Monitor encoder feedback non-blocking
    elif state == 'WAITING_WHEEL_ORIENTATION':
        if is_steering_aligned(wheels_config, target_angles_rad):
            state = 'SENDING_PULSE'

    # 3. State: SENDING_PULSE - Apply low-pass filter and power the traction motors
    elif state == 'SENDING_PULSE':
        if elapsed_s < pulse_duration_s:
            # Apply exponential low-pass filter
            filtered_vx_ms = (alpha_v * target_vx_ms) + ((1.0 - alpha_v) * filtered_vx_ms)
            filtered_vy_ms = (alpha_v * target_vy_ms) + ((1.0 - alpha_v) * filtered_vy_ms)
            filtered_wz_rad_s = (alpha_wz * target_wz_rad_s) + ((1.0 - alpha_wz) * filtered_wz_rad_s)

            # SIMPLIFIED KINETIC DISTRIBUTION (Exclusive pulse-wait modes)
            target_speeds_ms = {}
            for name, config in wheels_config.items():
                if abs(target_wz_rad_s) > 0.0:
                    # Pure rotation: V = R * w
                    wheel_radius_from_center_m = math.hypot(config['x'], config['y'])
                    target_speeds_ms[name] = wheel_radius_from_center_m * filtered_wz_rad_s
                elif abs(target_vy_ms) > 0.0:
                    # Pure Crab: V = Vy / sin(60)
                    target_speeds_ms[name] = filtered_vy_ms / math.sin(math.radians(60.0))
                else:
                    # Pure Straight: V = Vx
                    target_speeds_ms[name] = filtered_vx_ms

            send_hardware_command(wheel_pubs, target_angles_rad, target_speeds_ms, wheel_radius_m, wheels_config)

        else:
            # End of timer: Trigger hard braking, reset to WAITING
            send_hardware_command(wheel_pubs, target_angles_rad, zero_speeds_ms, wheel_radius_m, wheels_config)
            state = 'WAITING'

    return state, filtered_vx_ms, filtered_vy_ms, filtered_wz_rad_s