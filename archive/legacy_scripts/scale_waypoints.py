import numpy as np
import os

def scale_waypoints(path, factor=0.125):
    if not os.path.exists(path):
        print(f"File not found: {path}")
        return
    
    data = np.load(path)
    print(f"Original waypoints shape: {data.shape}")
    print(f"Original Sample [0]: {data[0]}")
    
    # Scale only X and Y (first 2 columns)
    data[:, 0] *= factor
    data[:, 1] *= factor
    
    # Do NOT scale Z if it's heading (rad)
    # Based on 1.57 value, it's definitely heading.
    
    new_path = path.replace(".npy", "_1x.npy")
    np.save(new_path, data)
    print(f"Scaled Sample [0]: {data[0]}")
    print(f"Saved to {new_path}")

if __name__ == "__main__":
    scale_waypoints("arcproLab/mdp/track_centerline.npy")
