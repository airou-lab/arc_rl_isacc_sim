import numpy as np
import os

# Get path relative to this script
current_dir = os.path.dirname(os.path.abspath(__file__))
track_path = os.path.join(current_dir, "../../arcproLab/mdp/track_centerline.npy")

data = np.load(track_path)
print(f"Loaded track data from: {track_path}")
print(f"Shape: {data.shape}")
print(f"First 10 waypoints (X, Y, Yaw):")
print(data[:10])

# Check min/max X and Y
print(f"X range: [{np.min(data[:, 0]):.2f}, {np.max(data[:, 0]):.2f}]")
print(f"Y range: [{np.min(data[:, 1]):.2f}, {np.max(data[:, 1]):.2f}]")
