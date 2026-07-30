import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Teleop ARCPro F1Tenth")
parser.add_argument("--num_envs", type=int, default=1)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)

import torch
import sys
import os
import pygame

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))
sys.path.insert(0, ROOT_DIR)

from isaaclab.envs import ManagerBasedRLEnv
from arcproLab.arcpro_env_cfg import ARCProEnvCfg

def main():
    # Initialize PyGame for keyboard input
    pygame.init()
    screen = pygame.display.set_mode((300, 200))
    pygame.display.set_caption("ARCPro Teleop")
    print("\n" + "="*50)
    print("TELEOP INSTRUCTIONS:")
    print("Click on the small pygame window that opened!")
    print("W / S : Forward / Brake")
    print("A / D : Steer Left / Right")
    print("="*50 + "\n")

    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.enable_cameras = False
    env_cfg.__post_init__()

    env = ManagerBasedRLEnv(cfg=env_cfg, render_mode="human")
    env.reset()

    # Action tensor: [steer, throttle] in range [-1.0, 1.0]
    action = torch.zeros((env.num_envs, 2), device=env.device)
    
    # Throttle mapping: action * (-20) - 20 
    # For 0.0 velocity (stopped), we need action = -1.0
    action[:, 1] = -1.0 

    steer_val = 0.0
    throttle_val = -1.0

    while app_launcher.app.is_running():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                app_launcher.app.close()
                return

        keys = pygame.key.get_pressed()
        
        # Steering: A/D mapped to 1.0 / -1.0
        if keys[pygame.K_a]:
            steer_val = 1.0
        elif keys[pygame.K_d]:
            steer_val = -1.0
        else:
            steer_val = 0.0
            
        # Throttle: W/S mapped to 1.0 / -1.0
        if keys[pygame.K_w]:
            throttle_val = 1.0
        elif keys[pygame.K_s]:
            throttle_val = -1.0 # Brake
        else:
            throttle_val = -1.0 

        action[:, 0] = steer_val
        action[:, 1] = throttle_val

        env.step(action)

    env.close()

if __name__ == "__main__":
    main()
