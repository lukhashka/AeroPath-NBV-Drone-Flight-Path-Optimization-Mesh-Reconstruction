import open3d as o3d
import numpy as np

def find_next_best_views(point_cloud_path, distance_to_target=6.0):
    print("[Analyzer] Завантаження хмари точок...")
    pcd = o3d.io.read_point_cloud(point_cloud_path)
    
    num_original_points = len(pcd.points)
    if num_original_points == 0:
        print("[Error] Хмара точок порожня або файл не знайдено.")
        return []
    print(f"[Analyzer] Завантажено {num_original_points} точок.")

    # --- АВТОМАТИЧНИЙ МАСШТАБ ТА СТИСНЕННЯ (Voxel Downsampling) ---
    print("[Analyzer] Розрахунок масштабу сцени...")
    bbox = pcd.get_axis_aligned_bounding_box()
    extent = bbox.get_max_bound() - bbox.get_min_bound()
    max_dim = np.max(extent) # Максимальний габарит сцени
    
    # Визначаємо розмір вокселя як 0.8% від розміру всієї сцени.
    # Це гарантує стабільну кількість точок незалежно від одиниць COLMAP.
    voxel_size = max_dim * 0.008 
    
    print("[Analyzer] Оптимізація щільності хмари (Voxel Downsampling)...")
    pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)
    print(f"[Analyzer] Хмару стиснуто: {num_original_points} -> {len(pcd_down.points)} точок.")

    # Адаптуємо радіус пошуку під новий розмір вокселя
    search_radius = voxel_size * 2.0
    min_neighbors = 8  # Якщо воксель майже порожній — це дірка

    print("[Analyzer] Розрахунок нормалей для оптимізованої хмари...")
    pcd_down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=search_radius * 1.5, max_nn=30))
    
    points = np.asarray(pcd_down.points)
    
    print("[Analyzer] Пошук зон із низькою щільністю через KD-Tree...")
    pcd_tree = o3d.geometry.KDTreeFlann(pcd_down)
    low_density_indices = []
    
    # Тепер цей цикл пробіжиться миттєво, бо точок стало значно менше
    for i in range(len(points)):
        [_, idx, _] = pcd_tree.search_radius_vector_3d(points[i], search_radius)
        if len(idx) < min_neighbors:
            low_density_indices.append(i)
            
    if not low_density_indices:
        print("[Analyzer] Дірок не знайдено. Хмара покрита ідеально.")
        return []
    
    gap_pcd = pcd_down.select_by_index(low_density_indices)
    
    print("[Analyzer] Кластеризація знайдених дефектів (DBSCAN)...")
    labels = np.array(gap_pcd.cluster_dbscan(eps=search_radius * 2.0, min_points=4, print_progress=False))
    max_label = labels.max()
    
    if max_label < 0:
        print("[Analyzer] Не вдалося сформувати стабільні кластери дірок.")
        return []

    gap_points = np.asarray(gap_pcd.points)
    gap_normals = np.asarray(gap_pcd.normals)
    nbv_waypoints = []
    
    # Якщо модель COLMAP не в метрах, distance_to_target теж треба масштабувати
    # Прив'яжемо дистанцію зйомки до динамічного розміру вокселя
    scaled_distance = voxel_size * 15.0 
    
    for cluster_id in range(max_label + 1):
        cluster_mask = (labels == cluster_id)
        if not np.any(cluster_mask):
            continue
            
        cluster_center = gap_points[cluster_mask].mean(axis=0)
        cluster_normal = gap_normals[cluster_mask].mean(axis=0)
        
        norm_length = np.linalg.norm(cluster_normal)
        if norm_length < 1e-5:
            cluster_normal = np.array([0.0, 0.0, 1.0])
        else:
            cluster_normal /= norm_length
        
        # Розрахунок позиції NBV
        drone_position = cluster_center + (cluster_normal * scaled_distance)
        look_at_direction = -cluster_normal
        
        nbv_waypoints.append({
            "cluster_id": cluster_id,
            "drone_XYZ": drone_position,
            "look_at": look_at_direction,
            "target_XYZ": cluster_center
        })
        
    print(f"[Analyzer] Згенеровано {len(nbv_waypoints)} первинних точок NBV.")
    return nbv_waypoints