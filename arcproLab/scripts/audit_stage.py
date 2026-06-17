
import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch
import os
import sys
from pxr import Usd, UsdGeom

# Add arcproLab to path
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ARCPRO_LAB_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
if ARCPRO_LAB_DIR not in sys.path:
    sys.path.append(ARCPRO_LAB_DIR)

from arcpro_env_cfg import ARCProEnvCfg
from isaaclab.envs import ManagerBasedRLEnv

def main():
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = 1
    env_cfg.enable_cameras = True
    env = ManagerBasedRLEnv(cfg=env_cfg)
    
    # Get the stage directly from the simulation app's context
    import omni.usd
    stage = omni.usd.get_context().get_stage()
    
    print("\n" + "="*60)
    print("STAGE HIERARCHY AUDIT")
    print("="*60)
    
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if "/Robot" in path:
            print(f"Prim: {path} [{prim.GetTypeName()}]")

    simulation_app.close()

if __name__ == "__main__":
    main()
