import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Check action space.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import os
import sys
from isaaclab.envs import ManagerBasedRLEnv
# Add arcproLab to path
sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
from arcpro_env_cfg import ARCProEnvCfg

def main():
    env_cfg = ARCProEnvCfg()
    env = ManagerBasedRLEnv(cfg=env_cfg)
    print(f"\nACTION SPACE: {env.action_space}\n")
    env.close()
    simulation_app.close()

if __name__ == "__main__":
    main()
