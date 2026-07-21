import torch
B = 8
vel = torch.rand(B)
go_signal = torch.ones(B, 1)
cond1 = vel < 0.3
cond2 = go_signal > 0.5
res = cond1 & cond2
print("Shape of res:", res.shape)
