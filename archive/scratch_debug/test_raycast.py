import os
import torch
import math
from isaaclab.app import AppLauncher
import argparse

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from isaaclab.envs import ManagerBasedEnv
from arcproLab.arcpro_env_cfg import ARCProEnvCfg

env_cfg = ARCProEnvCfg()
env_cfg.scene.num_envs = 1
env_cfg.enable_cameras = False
env_cfg.__post_init__()

env = ManagerBasedEnv(cfg=env_cfg)

import torch
action = torch.zeros((env.num_envs, 2), device=env.device)
env.step(action)

import omni.physx
query = omni.physx.get_physx_scene_query_interface()
world_x, world_y = -16.197, 5.50
hit = query.raycast_closest((world_x, world_y, 100.0), (0.0, 0.0, -1.0), 200.0)

print(f"RAYCAST HIT: {hit['hit']}")
if hit['hit']:
    print(f"POSITION: {hit['position']}")

hits = query.raycast_all((world_x, world_y, 100.0), (0.0, 0.0, -1.0), 200.0)
print(f"RAYCAST ALL HITS: {len(hits)}")
for h in hits:
    print(f"HIT: {h['position']} | BODY: {h['rigidBody']} | COLLIDER: {h['collider']}")


simulation_app.close()
