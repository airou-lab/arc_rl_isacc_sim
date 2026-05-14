import numpy as np
import os

wp_path = 'arcproLab/mdp/track_centerline_1x.npy'
if not os.path.exists(wp_path):
    print("Waypoint file not found.")
    exit(1)

wp = np.load(wp_path)
n = len(wp)
min_dist = 1000.0

# Only check every 10th point to speed up if needed, but here I'll check all
for i in range(n):
    # Distance to all other points
    dists = np.linalg.norm(wp[i, :2] - wp[:, :2], axis=1)
    
    # Ignore points within 50 waypoints (avoiding sequence)
    mask = np.ones(n, dtype=bool)
    start = (i - 50) % n
    end = (i + 50) % n
    if start < end:
        mask[start:end] = False
    else:
        mask[start:] = False
        mask[:end] = False
        
    dists = dists[mask]
    if len(dists) > 0:
        local_min = np.min(dists)
        if local_min < min_dist:
            min_dist = local_min

print(f"Min distance between non-adjacent waypoints: {min_dist:.3f}m")
