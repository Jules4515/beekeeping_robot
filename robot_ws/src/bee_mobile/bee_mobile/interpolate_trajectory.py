#!/usr/bin/env python3
import yaml
import numpy as np
from pyproj import Transformer

# --- CONFIGURATION ---
INPUT_FILE = 'src/bee_mobile/config/waypoints_GPS_copy.yaml'
OUTPUT_FILE = 'src/bee_mobile/config/waypoints_GPS_interpolated.yaml'
TARGET_DISTANCE_METERS = 1.0  # Distance souhaitée entre chaque nouveau point (en mètres)

def interpolate_trajectory():
    # Load raw dense waypoints
    with open(INPUT_FILE, 'r') as f:
        data = yaml.safe_load(f)
    
    waypoints = data.get('waypoints_GPS', [])
    if not waypoints:
        print("Error: No waypoints found in the file.")
        return

    lats = np.array([wp['latitude'] for wp in waypoints])
    lons = np.array([wp['longitude'] for wp in waypoints])
    yaws = np.array([wp['yaw'] for wp in waypoints])

    # Create a dynamic local Cartesian projection centered on the starting point
    # This avoids UTM zone boundary errors and provides accurate metric distances
    proj_string = f"+proj=aeqd +lat_0={lats[0]} +lon_0={lons[0]} +datum=WGS84 +units=m"
    transformer_to_cart = Transformer.from_crs("EPSG:4326", proj_string, always_xy=True)
    transformer_to_gps = Transformer.from_crs(proj_string, "EPSG:4326", always_xy=True)

    # Convert GPS to Local Cartesian (X, Y in meters)
    x, y = transformer_to_cart.transform(lons, lats)

    # Compute continuous cumulative distance along the path
    dx = np.diff(x)
    dy = np.diff(y)
    distances = np.sqrt(dx**2 + dy**2)
    cum_dist = np.insert(np.cumsum(distances), 0, 0.0)
    total_dist = cum_dist[-1]

    # Generate new target distances based on the user-defined metric step
    new_cum_dist = np.arange(0, total_dist, TARGET_DISTANCE_METERS)
    
    # Ensure the final point is included if it's reasonably far from the last step
    if total_dist - new_cum_dist[-1] > (TARGET_DISTANCE_METERS * 0.2):
        new_cum_dist = np.append(new_cum_dist, total_dist)

    # Linear interpolation for X and Y coordinates
    new_x = np.interp(new_cum_dist, cum_dist, x)
    new_y = np.interp(new_cum_dist, cum_dist, y)

    # Unwrap yaw angles to handle -180/180 degree singularities before interpolating
    yaws_rad = np.radians(yaws)
    yaws_unwrapped = np.unwrap(yaws_rad)
    new_yaws_unwrapped = np.interp(new_cum_dist, cum_dist, yaws_unwrapped)
    
    # Convert back to degrees and normalize to [-180, 180] range
    new_yaws = np.degrees(new_yaws_unwrapped)
    new_yaws = (new_yaws + 180) % 360 - 180

    # Convert interpolated Cartesian coordinates back to Global GPS
    new_lons, new_lats = transformer_to_gps.transform(new_x, new_y)

    # Write the mathematically reduced dataset matching the original formatting
    print(f"Original points: {len(x)}")
    print(f"Interpolated points: {len(new_x)} (Step: {TARGET_DISTANCE_METERS}m)")

    with open(OUTPUT_FILE, 'w') as f:
        f.write("waypoints_GPS:\n")
        for i in range(len(new_x)):
            f.write(f"- name      : WP_{i+1}\n")
            f.write(f"  latitude  : {new_lats[i]:.7f}\n")
            f.write(f"  longitude : {new_lons[i]:.7f}\n")
            f.write(f"  yaw       : {new_yaws[i]:.2f}\n")
            f.write(f"  wait_time : 0.0\n\n")

if __name__ == '__main__':
    interpolate_trajectory()