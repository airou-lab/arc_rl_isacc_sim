import torch
B = 8
reward_buf = torch.zeros(B)
val = torch.ones(B, B)
try:
    reward_buf += val
    print("NO CRASH! sum:", reward_buf.sum())
except Exception as e:
    print("CRASH:", e)
