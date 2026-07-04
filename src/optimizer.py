import numpy as np
from scipy.spatial import distance_matrix
from scipy.interpolate import make_interp_spline

def optimize_drone_path(waypoints, smoothness=3):
    print("[Optimizer] Оптимізація маршруту (Завдання комівояжера)...")
    points = np.array(waypoints)
    num_points = len(points)
    
    if num_points <= 2:
        return points, points
        
    dist_mat = distance_matrix(points, points)
    unvisited = list(range(1, num_points))
    current_idx = 0
    tour = [current_idx]
    
    # Алгоритм найближчого сусіда
    while unvisited:
        next_idx = min(unvisited, key=lambda x: dist_mat[current_idx][x])
        unvisited.remove(next_idx)
        tour.append(next_idx)
        current_idx = next_idx
        
    ordered_points = points[tour]
    
    print("[Optimizer] Гіпер-параметричне згладжування траєкторії сплайнами...")
    t = np.linspace(0, 1, len(ordered_points))
    t_smooth = np.linspace(0, 1, len(ordered_points) * 10)
    
    # 'clamped' фіксує нульове прискорення на кінцях маршруту
    spline = make_interp_spline(t, ordered_points, k=smoothness, bc_type='clamped')
    smooth_trajectory = spline(t_smooth)
    
    return ordered_points, smooth_trajectory