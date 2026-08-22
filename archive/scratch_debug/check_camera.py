import torch
import numpy as np
import cv2

from isaaclab.envs import ManagerBasedRLEnv
from arcproLab.arcpro_env_cfg import ARCProEnvCfg

def main():
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = 1
    
    env = ManagerBasedRLEnv(cfg=env_cfg)
    
    env.reset()
    for i in range(10):
        env.step(torch.zeros((1, 2), device=env.device))
        
    cam = env.scene.sensors["tiled_camera"]
    img = cam.data.output["rgb"][0].cpu().numpy()
    
    print("Shape:", img.shape)
    print("Dtype:", img.dtype)
    print("Min:", np.min(img))
    print("Max:", np.max(img))
    print("Mean:", np.mean(img))

if __name__ == "__main__":
    main()
