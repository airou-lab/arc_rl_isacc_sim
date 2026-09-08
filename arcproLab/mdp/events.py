import math
import torch
from isaaclab.managers import SceneEntityCfg
from isaaclab.envs import ManagerBasedRLEnv
from mdp.curriculum_config import get_sector_thresholds

def reset_robot_curriculum_spawn(env: ManagerBasedRLEnv, env_ids: torch.Tensor, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")):
    """
    5-Sector Certified Circuit Spawner (Configurable per sector):
    Sector 1: Start Line (X = -16.197, Y = 5.50, yaw = -1.5708 [South])
    Sector 2: Corner 1 Runway Approach (X = -16.197, Y = -3.50, yaw = -1.5708 [South])
    Sector 3: Leg 2 Post-C1 Exit (X = -12.000, Y = -7.10, yaw = 0.0 [East])
    Sector 4: Leg 2 Intersection Approach (X = -8.000, Y = -7.10, yaw = 0.0 [East])
    Sector 5: North Straight Return Corridor (X = -6.082, Y = 6.377, yaw = 3.14159 [West])
    Applies ±5cm lateral jitter and ±3° heading noise to prevent open-loop memorization.
    """
    if env_ids is None:
        env_ids = torch.arange(env.scene.num_envs, device=env.device)
    asset = env.scene[asset_cfg.name]
    num_resets = len(env_ids)

    env_origins = env.scene.env_origins[env_ids]
    final_pos = torch.zeros((num_resets, 3), device=env.device)
    quats = torch.zeros((num_resets, 4), device=env.device)

    # 5-Sector Certified Circuit Spawner with Dynamic Curriculum Thresholds:
    t1, t2, t3, t4 = get_sector_thresholds()
    r = torch.rand(num_resets, device=env.device)
    base_x = torch.where(r < t1, torch.tensor(-16.197, device=env.device),
             torch.where(r < t2, torch.tensor(-16.197, device=env.device),
             torch.where(r < t3, torch.tensor(-12.000, device=env.device),
             torch.where(r < t4, torch.tensor(-8.000, device=env.device),
                                  torch.tensor(-6.082, device=env.device)))))

    base_y = torch.where(r < t1, torch.tensor(5.50, device=env.device),
             torch.where(r < t2, torch.tensor(-3.50, device=env.device),
             torch.where(r < t3, torch.tensor(-7.10, device=env.device),
             torch.where(r < t4, torch.tensor(-7.10, device=env.device),
                                  torch.tensor(6.377, device=env.device)))))

    base_yaw = torch.where(r < t1, torch.tensor(-1.5708, device=env.device),
               torch.where(r < t2, torch.tensor(-1.5708, device=env.device),
               torch.where(r < t3, torch.tensor(0.0, device=env.device),
               torch.where(r < t4, torch.tensor(0.0, device=env.device),
                                    torch.tensor(3.14159265, device=env.device)))))

    # Domain Randomization: Lateral Jitter (±5cm) & Heading Noise (±3 deg)
    # Prevents fixed-trajectory memorization and forces active visual lane centering!
    lat_jitter = (torch.rand(num_resets, device=env.device) * 2.0 - 1.0) * 0.05
    head_noise = (torch.rand(num_resets, device=env.device) * 2.0 - 1.0) * (3.0 * math.pi / 180.0)

    # Apply lateral jitter perpendicular to track heading
    # Heading is South (-1.5708 rad): lateral normal is (-sin(yaw), cos(yaw)) = (1, 0)
    spawn_x = base_x - torch.sin(base_yaw) * lat_jitter
    spawn_y = base_y + torch.cos(base_yaw) * lat_jitter
    spawn_yaw = base_yaw + head_noise

    final_pos[:, 0] = env_origins[:, 0] + spawn_x
    final_pos[:, 1] = env_origins[:, 1] + spawn_y
    final_pos[:, 2] = env_origins[:, 2] + 0.12 # Safe drop height

    half_yaw = spawn_yaw / 2.0
    quats[:, 0] = torch.cos(half_yaw)
    quats[:, 1] = 0.0
    quats[:, 2] = 0.0
    quats[:, 3] = torch.sin(half_yaw)

    velocities = torch.zeros((num_resets, 6), device=env.device)

    # Teleport and Apply Initial State
    asset.write_root_pose_to_sim(torch.cat([final_pos, quats], dim=-1), env_ids=env_ids)
    asset.write_root_velocity_to_sim(velocities, env_ids=env_ids)

    # Zero joint states
    joint_pos = asset.data.default_joint_pos[env_ids]
    joint_vel = torch.zeros_like(asset.data.joint_vel[env_ids])
    asset.write_joint_state_to_sim(joint_pos, joint_vel, env_ids=env_ids)

    if "distance" in env.extras:
        env.extras["distance"][env_ids] = 0.0

    if "prev_action_jerk" in env.extras:
        env.extras["prev_action_jerk"][env_ids] = 0.0
    if "prev_action_smoothness" in env.extras:
        env.extras["prev_action_smoothness"][env_ids] = 0.0

    try:
        from mdp.go_signal_manager import get_go_signal_manager
        mgr = get_go_signal_manager(num_envs=env.num_envs, device=env.device)
        mgr.reset(env_ids)
    except Exception:
        pass
