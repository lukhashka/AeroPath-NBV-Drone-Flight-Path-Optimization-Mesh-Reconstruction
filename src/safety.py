import open3d as o3d
import numpy as np

def build_mesh_poisson(point_cloud_path, depth=9):
    print("[Safety] Запуск Poisson Surface Reconstruction...")
    pcd = o3d.io.read_point_cloud(point_cloud_path)
    pcd.estimate_normals()
    
    # Реконструкція Поассона
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=depth)
    
    # Очищення мешу від низькощільних артефактів на краях
    vertices_to_remove = densities < np.quantile(densities, 0.05)
    mesh.remove_vertices_by_mask(vertices_to_remove)
    return mesh

def filter_safe_waypoints(mesh_legacy, raw_waypoints, min_alt=5.0, max_alt=120.0, safety_distance=4.0):
    print("[Safety] Фільтрація точок через Tensor Raycasting...")
    # Перетворення в тензорний формат для прискорення
    t_mesh = o3d.t.geometry.TriangleMesh.from_legacy(mesh_legacy)
    scene = o3d.t.geometry.RaycastingScene()
    _ = scene.add_triangles(t_mesh)
    
    safe_waypoints = []
    
    for wp in raw_waypoints:
        drone_xyz = wp["drone_XYZ"]
        target_xyz = wp["target_XYZ"]
        
        # Фільтр 1: Висота польоту (Z)
        if not (min_alt <= drone_xyz[2] <= max_alt):
            continue
            
        # Фільтр 2: Відстань до перешкод (Колізія)
        query_point = o3d.core.Tensor([drone_xyz], dtype=o3d.core.Dtype.Float32)
        dist_to_mesh = scene.compute_distance(query_point).numpy()[0]
        if dist_to_mesh < safety_distance:
            continue
            
        # Фільтр 3: Перевірка видимості (Оклюзія)
        ray_dir = target_xyz - drone_xyz
        dist_to_target = np.linalg.norm(ray_dir)
        ray_dir_norm = ray_dir / dist_to_target
        
        ray = o3d.core.Tensor([[
            drone_xyz[0], drone_xyz[1], drone_xyz[2],
            ray_dir_norm[0], ray_dir_norm[1], ray_dir_norm[2]
        ]], dtype=o3d.core.Dtype.Float32)
        
        ray_hit = scene.cast_rays(ray)
        t_hit = ray_hit['t_hit'].numpy()[0]
        
        # Якщо промінь врізався в рельєф/об'єкт значно раніше, ніж долетів до цілі
        if t_hit < (dist_to_target - 0.2):
            continue
            
        safe_waypoints.append(wp)
        
    return safe_waypoints