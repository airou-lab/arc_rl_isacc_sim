import numpy as np

cache_path = "arcproLab/mdp/track_boundaries_1x.npz"
wp_path = "arcproLab/mdp/track_centerline_1x.npy"

if True:
    data = np.load(cache_path)
    white = data['white']
    yellow = data['yellow']
    gate = data['gate']
    
    wps = np.load(wp_path)
    
    # For each white point, find dist to closest waypoint
    # We can do this in chunks to save memory
    new_white = []
    new_gate = list(gate) if gate.size > 0 else []
    
    for pt in white:
        dists = np.linalg.norm(wps[:, :2] - pt[:2], axis=1)
        if np.min(dists) < 0.8: # If within 0.8m of centerline, it's a gate
            new_gate.append(pt)
        else:
            new_white.append(pt)
            
    np.savez(cache_path, white=np.array(new_white), yellow=yellow, gate=np.array(new_gate))
    print("Fixed cache!")
