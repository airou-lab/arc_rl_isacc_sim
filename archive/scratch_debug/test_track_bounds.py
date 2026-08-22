import os
import sys
import torch

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

# Get Track Bounding Box
import omni.usd
from pxr import UsdGeom
stage = omni.usd.get_context().get_stage()
track_prim = stage.GetPrimAtPath("/World/envs/env_0/Track")
if track_prim.IsValid():
    bbox_cache = UsdGeom.BBoxCache(0.0, ["default"])
    bound = bbox_cache.ComputeWorldBound(track_prim)
    range_ = bound.ComputeAlignedBox()
    print("Track Bounds: Min", range_.GetMin(), "Max", range_.GetMax())
else:
    print("Track Prim NOT FOUND")

simulation_app.close()
