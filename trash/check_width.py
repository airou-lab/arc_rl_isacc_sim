import numpy as np
data = np.load('arcproLab/mdp/track_boundaries_1x.npz')
white = data['white']
wp = np.load('arcproLab/mdp/track_centerline_1x.npy')

# Pick a waypoint
p = wp[100, :2]
# Find white points near it (within 1m)
dists = np.linalg.norm(white[:, :2] - p, axis=1)
nearby = white[dists < 1.0]

if len(nearby) > 0:
    # Project nearby points onto the normal of the waypoint to see lateral spread
    yaw = wp[100, 2]
    normal = np.array([np.sin(yaw), -np.cos(yaw)])
    
    # lateral offset from waypoint
    lat_offsets = np.dot(nearby[:, :2] - p, normal)
    
    # The white points should form two clusters (left and right line)
    # or just one if it's a one-way track.
    # Let's see the range of offsets
    print(f"Lateral offsets range: {np.min(lat_offsets):.3f} to {np.max(lat_offsets):.3f}m")
    
    # Let's find the 'thickness' of one line
    # Sort offsets and look for gaps
    lat_offsets.sort()
    diffs = np.diff(lat_offsets)
    gaps = np.where(diffs > 0.05)[0] # 5cm gap between lines
    
    if len(gaps) > 0:
        # We have at least two lines
        start = 0
        for gap_idx in gaps:
            cluster = lat_offsets[start:gap_idx+1]
            width = np.max(cluster) - np.min(cluster)
            print(f"Line cluster width: {width:.3f}m")
            start = gap_idx + 1
        # last cluster
        cluster = lat_offsets[start:]
        width = np.max(cluster) - np.min(cluster)
        print(f"Line cluster width: {width:.3f}m")
    else:
        width = np.max(lat_offsets) - np.min(lat_offsets)
        print(f"Single line cluster width: {width:.3f}m")
else:
    print("No white points found near waypoint 100")
