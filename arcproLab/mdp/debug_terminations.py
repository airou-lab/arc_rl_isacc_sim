import torch
from isaaclab.managers import SceneEntityCfg
from isaaclab.envs import ManagerBasedRLEnv

def debug_termination(env: ManagerBasedRLEnv) -> torch.Tensor:
    # 1. Physical Crash
    sensor = env.scene.sensors["contact_forces"]
    forces = torch.norm(sensor.data.net_forces_w, dim=-1)
    chassis_crash = torch.any(forces > 1.0, dim=1)
    
    # 2. Roadmark hit
    obs = env.observation_manager.compute()["policy"]
    lat_err = torch.abs(obs[:, 8])
    marker_hit = lat_err > 0.275
    
    if chassis_crash.any():
        # Get the names of what we hit
        # Note: In Isaac Lab, we can look at the raw contact report if needed
        print(f"[DEBUG] CHASSIS CRASH! Max Force: {torch.max(forces):.2f}")
        
    if marker_hit.any():
        print(f"[DEBUG] MARKER HIT! LatErr Norm: {lat_err[0]:.4f} (Target > 0.275)")
        
    return chassis_crash | marker_hit
