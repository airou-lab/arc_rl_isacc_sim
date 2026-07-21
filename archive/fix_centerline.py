import numpy as np

# Load boundaries
data = np.load('arcproLab/mdp/track_boundaries_1x.npz')
yellow = data['yellow']
white = data['white']

# We just need to generate a path starting near the car: X=-16.2, Y=5.5
# and heading South (-Y).
# Let's find midpoints
mids = []
for py in yellow:
    dists = np.linalg.norm(white[:, :2] - py[:2], axis=1)
    idx = np.argmin(dists)
    if dists[idx] < 1.0:
        mids.append((py[:2] + white[idx, :2]) / 2.0)

mids = np.unique(np.round(np.array(mids), 3), axis=0)

# Start near car
start_p = np.array([-16.197, 5.50])
dists = np.linalg.norm(mids - start_p, axis=1)
curr_idx = np.argmin(dists)

chain = [mids[curr_idx]]
visited = {curr_idx}

# Since we want to go South, we need to pick the next point that has a smaller Y (or just generally along the path).
# Let's use a directional penalty to encourage going South initially.
dir_vec = np.array([0.0, -1.0])

for _ in range(len(mids)):
    curr_p = chain[-1]
    dists = np.linalg.norm(mids - curr_p, axis=1)
    
    # Penalize points behind us
    vecs = mids - curr_p
    vecs_norm = np.linalg.norm(vecs, axis=1)
    vecs_norm[vecs_norm == 0] = 1.0
    dirs = vecs / vecs_norm[:, None]
    
    dots = np.sum(dirs * dir_vec, axis=1)
    
    # We want dots close to 1
    scores = dists - dots * 0.5 # Encourage moving in dir_vec
    scores[list(visited)] = 1e6
    scores[dists > 2.0] = 1e6
    
    next_idx = np.argmin(scores)
    if scores[next_idx] > 1e5:
        break
        
    chain.append(mids[next_idx])
    visited.add(next_idx)
    
    # Update dir_vec for next step (momentum)
    dir_vec = dirs[next_idx]

chain = np.array(chain)
print("Chain length:", len(chain))

# Add yaw
final_data = []
for i in range(len(chain) - 1):
    p1, p2 = chain[i], chain[i+1]
    yaw = np.arctan2(p2[1] - p1[1], p2[0] - p1[0])
    final_data.append([p1[0], p1[1], yaw])
    
# duplicate last point
final_data.append([chain[-1][0], chain[-1][1], final_data[-1][2]])

final_data = np.array(final_data)
np.save('arcproLab/mdp/track_centerline.npy', final_data)
print("Saved new track_centerline.npy!")
