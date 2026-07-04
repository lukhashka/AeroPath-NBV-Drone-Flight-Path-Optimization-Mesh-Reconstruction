import sys
import numpy as np
# Importing custom package modules from the src tree
from src.analyzer import find_next_best_views
from src.safety import build_mesh_poisson, filter_safe_waypoints
from src.optimizer import optimize_drone_path
from src.exporter import export_to_dji_kmz, export_to_qgc_plan
from src.visualizer import visualize_mission

def local_enu_to_wgs84(x, y, z, home_lat, home_lon, home_alt, scale_factor=1.0):
    """
    Converts local COLMAP coordinates (X-East, Y-North, Z-Up) to global WGS84 coordinates.
    Employs WGS84 ellipsoid curvature radii calculations for high geodetic precision.
    """
    # Scale abstract model coordinates into actual meters
    x_meters = x * scale_factor
    y_meters = y * scale_factor
    z_meters = z * scale_factor
    
    # WGS84 Ellipsoid constants
    a = 6378137.0
    e_sq = 0.00669437999014
    
    lat_rad = np.radians(home_lat)
    
    # Compute the radii of curvature for the given latitude
    sin_lat_sq = np.sin(lat_rad) ** 2
    rm = a * (1 - e_sq) / ((1 - e_sq * sin_lat_sq) ** 1.5)
    rn = a / np.sqrt(1 - e_sq * sin_lat_sq)
    
    # Compute geographic angular offsets in degrees
    delta_lat = (y_meters / rm) * (180.0 / np.pi)
    delta_lon = (x_meters / (rn * np.cos(lat_rad))) * (180.0 / np.pi)
    
    lat = home_lat + delta_lat
    lon = home_lon + delta_lon
    alt = home_alt + z_meters  # Relative altitude stacked on top of home base altitude
    
    return lat, lon, alt

def run_pipeline(pcd_path, output_prefix="colmap_mission"):
    print("=== Step 1: Point Cloud Analysis & Gap Detection ===")
    # Extract structural gap centroids and their initial Next-Best-View configurations
    raw_waypoints = find_next_best_views(point_cloud_path=pcd_path)
    
    if not raw_waypoints:
        print("No structural coverage gaps detected. Termination triggered.")
        return

    print("\n=== Step 2: Mesh Reconstruction & Safety Collision Filters ===")
    mesh = build_mesh_poisson(pcd_path)
    
    # Bound the volumetric boundaries to establish dynamic metrics scaling
    bbox = mesh.get_axis_aligned_bounding_box()
    max_dim = np.max(bbox.get_max_bound() - bbox.get_min_bound())
    voxel_size = max_dim * 0.008
    adaptive_safety_distance = voxel_size * 3.0
    
    # Process views against a fast Tensor Raycasting Scene to prevent drone collisions
    safe_waypoints = filter_safe_waypoints(
        mesh_legacy=mesh, 
        raw_waypoints=raw_waypoints, 
        min_alt=-10000.0, 
        max_alt=10000.0, 
        safety_distance=adaptive_safety_distance
    )
    print(f"Out of {len(raw_waypoints)} initial waypoints, {len(safe_waypoints)} cleared the safety boundary filter.")

    if not safe_waypoints:
        print("Zero waypoints passed the geometric safety validation.")
        return

    print("\n=== Step 3: Route Optimization & Kinematic Path Smoothing ===")
    xyz_points = [wp["drone_XYZ"] for wp in safe_waypoints]
    
    # Compute the optimal path sorting sequence (TSP) and construct interpolation splines
    ordered_points, smooth_trajectory = optimize_drone_path(xyz_points, smoothness=3)

    print("\n=== Step 4: Native Flight Mission Export ===")
    # Target geographical location anchors (New Jersey, USA structural asset)
    home_lat = 41.048678
    home_lon = -74.006744
    home_alt = 35.0  # Elevation AMSL (Above Mean Sea Level) in meters
    
    # Manual origin calibration offset adjustments in meters to handle non-aligned SfM local centers
    bias_x = 120.0  
    bias_y = 50.0   
    
    # Dynamic automated scale adjustments for unscaled arbitrary COLMAP units
    if max_dim < 20.0:
        scale_factor = 180.0 / max_dim
        print(f"[Scale] Unscaled arbitrary units detected. Auto-scaling ratio applied: x{scale_factor:.2f}")
    else:
        scale_factor = 1.0
        print("[Scale] Dataset features proper metric scaling (scale_factor = 1.0)")
    
    final_mission_data = []
    for pt in ordered_points:
        for original_wp in safe_waypoints:
            if np.allclose(original_wp["drone_XYZ"], pt, atol=1e-3):
                
                # Derive precise camera gimbal tilt angles (pitch) via look_at normal components
                look_at = original_wp["look_at"]
                denom = np.linalg.norm(look_at)
                if denom > 1e-5:
                    calculated_pitch = np.degrees(np.arcsin(look_at[2] / denom))
                else:
                    calculated_pitch = -45.0
                pitch = float(np.clip(calculated_pitch, -90.0, 0.0))
                
                # Execute geodetic conversion calculations
                lat, lon, alt = local_enu_to_wgs84(
                    x=(pt[0] * scale_factor) + bias_x, 
                    y=(pt[1] * scale_factor) + bias_y, 
                    z=pt[2] * scale_factor,
                    home_lat=home_lat, home_lon=home_lon, home_alt=home_alt,
                    scale_factor=1.0
                )
                
                # Calculate relative altitude to feed native flight plan controllers
                relative_alt = float(pt[2] * scale_factor)
                
                final_mission_data.append((lat, lon, relative_alt, pitch))
                break

    # Commit localized output structures onto standard industrial telemetry files
    export_to_dji_kmz(final_mission_data, f"{output_prefix}_dji.kmz")
    export_to_qgc_plan(final_mission_data, f"{output_prefix}_qgc.plan")
    
    print(f"\n[SUCCESS] Pipeline executed successfully! Mission assets mapped and saved:")
    print(f" -> {output_prefix}_dji.kmz (Native DJI Pilot 2 WPML Archive)")
    print(f" -> {output_prefix}_qgc.plan (QGroundControl MAVLink Mission Schema)")

    print("\n=== Step 5: Render Results ===")
    # Spawns the OpenGL interactive 3D rendering view frame
    visualize_mission(mesh, ordered_points, smooth_trajectory)

if __name__ == "__main__":
    pcd_path = "data/fused2.ply" 
    run_pipeline(pcd_path, output_prefix="colmap_mission")