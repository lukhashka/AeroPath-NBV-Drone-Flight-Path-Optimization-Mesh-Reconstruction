import open3d as o3d
import numpy as np

def visualize_mission(mesh, ordered_points, smooth_trajectory):
    print("[Visualizer] Ініціалізація 3D вікна візуалізації...")
    
    # 1. Налаштовуємо відображення мешу поверхні
    mesh.compute_vertex_normals()
    mesh.paint_uniform_color([0.6, 0.6, 0.6]) # Спокійний сірий колір для рельєфу
    
    # Масив, куди ми збиратимемо всі 3D об'єкти для сцени
    geometries = [mesh]
    
    # 2. Будуємо лінію маршруту (LineSet)
    # Зв'язуємо точки плавної траєкторії послідовними індексами: [[0,1], [1,2], [2,3]...]
    lines = [[i, i + 1] for i in range(len(smooth_trajectory) - 1)]
    colors = [[0.0, 0.8, 0.0] for _ in range(len(lines))] # Яскраво-зелений колір шляху
    
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(smooth_trajectory)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)
    geometries.append(line_set)
    
    # 3. Додаємо вейпоінти як червоні 3D-сфери
    # Рахуємо масштаб сцени, щоб розмір сфер автоматично адаптувався під модель
    bbox = mesh.get_axis_aligned_bounding_box()
    max_dim = np.max(bbox.get_max_bound() - bbox.get_min_bound())
    sphere_radius = max_dim * 0.012 # Сфери займатимуть трохи більше 1% від розміру сцени
    
    for pt in ordered_points:
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=sphere_radius)
        sphere.compute_vertex_normals()
        sphere.paint_uniform_color([1.0, 0.1, 0.1]) # Яскраво-червоні точки зупинки дрона
        sphere.translate(pt) # Зміщуємо сферу в координати вейпоінта
        geometries.append(sphere)
        
    # 4. Запускаємо вікно рендерингу
    print("[Visualizer] Відкриття вікна. Ротація: ЛКМ, Панорама: ПКМ, Зум: Скрол.")
    o3d.visualization.draw_geometries(
        geometries,
        window_name="Farsight Drone Path Optimizer - 3D View",
        width=1280,
        height=720,
        mesh_show_back_face=True # Показувати внутрішні стінки полігонів
    )