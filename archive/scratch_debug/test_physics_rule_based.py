import torch
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True, "enable_cameras": True})

from arcproLab.arcpro_env_cfg import ARCProEnvCfg
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.sensors.camera import Camera, CameraCfg

def main():
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = 1
    # Increase episode length so we can observe it
    env_cfg.episode_length_s = 100.0
    
    env = ManagerBasedRLEnv(cfg=env_cfg)
    
    obs, _ = env.reset()
    
    print("Testing rule-based driving...")
    print("Driving straight for 50 steps...")
    
    actions = torch.zeros((1, 2), device=env.device)
    
    # 0 = steering, 1 = drive
    for i in range(50):
        actions[:, 0] = 0.0 # straight
        actions[:, 1] = 0.5 # half throttle (approx 20 rad/s = 1 m/s)
        env.step(actions)
        
    print("Chassis pos after straight:", env.scene["robot"].data.root_pos_w)
    
    print("Turning right gently for 50 steps...")
    for i in range(50):
        actions[:, 0] = -0.5 # half steer right
        actions[:, 1] = 0.5 # half throttle
        env.step(actions)
        
    print("Chassis pos after turn:", env.scene["robot"].data.root_pos_w)
    print("Simulation complete. The physics are correct if it moved forward and then turned right.")

if __name__ == "__main__":
    main()
    simulation_app.close()
