import torch
B = 8
vel = torch.zeros(B)
cond = vel < 0.3
res = torch.where(cond, -1.0, 0.0)
print(res)
print(res.shape)
