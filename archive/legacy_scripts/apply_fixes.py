import numpy as np

# 1. Fix track_centerline.npy
# We will just shift it by -0.24 to match the new center, and reverse it so it goes South
wp_path = 'arcproLab/mdp/track_centerline.npy'
data = np.load(wp_path)
data[:, 0] -= 0.243 # Shift X from -15.96 to -16.20
# Reverse order
data = data[::-1].copy()
# Update Yaw to face South (-1.5707964) instead of North
data[:, 2] = -1.5707964
np.save(wp_path, data)
print("Fixed track_centerline.npy")

# 2. Fix gate points in track_boundaries_1x.npz
cache_path = 'arcproLab/mdp/track_boundaries_1x.npz'
boundaries = np.load(cache_path)
white = boundaries['white']
yellow = boundaries['yellow']
gate = boundaries['gate']

if gate.size > 0:
    # Scale gates from 8x back down to 1x
    gate = gate / 8.0
    np.savez(cache_path, white=white, yellow=yellow, gate=gate)
    print("Fixed gate scale in track_boundaries_1x.npz")
else:
    print("No gates found.")
