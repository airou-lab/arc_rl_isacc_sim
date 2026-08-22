import torch
num_wps = 1500
current_idx = torch.tensor([10, 10, 11, 10, 11, 1499, 0, 1])
prev_idx = torch.tensor(   [10, 11, 10, 11, 1499, 0, 1, 0])

delta = (current_idx - prev_idx) % num_wps
delta_real = torch.where(delta > num_wps // 2, delta - num_wps, delta)
print(delta_real)
