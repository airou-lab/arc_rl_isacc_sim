import os
import math
import torch
from isaaclab.managers import SceneEntityCfg
from isaaclab.envs import ManagerBasedRLEnv
from mdp.curriculum_config import get_sector_thresholds


def reset_robot_curriculum_spawn(
    env: ManagerBasedRLEnv,
    env_ids: torch.Tensor,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
):
    """
    5-Sector Certified Circuit Spawner with Corner Immersion:
    Sector 1: Start Line (X = -16.200, Y = 5.500, yaw = -1.5708 [South])
    Sector 2: Corner 1 Runway Approach (X = -16.218, Y = -4.800, yaw = -1.5708 [South, 0.8m runway into C1])
    Sector 3: Corner 1 Apex Immersion (X = -15.800, Y = -6.557, yaw = -0.8260 [-47.3 deg])
    Sector 4: Leg 2 Intersection Approach (X = -6.000, Y = -7.118, yaw = 0.0000 [East, 0.8m before turn])
    Sector 5: Corner 4 Runway Approach (X = -13.548, Y = 7.116, yaw = 3.14159 [West, 1.0m runway into C4])
    Applies ±1.0cm lateral jitter and ±2° heading noise to prevent open-loop memorization.
    """
    if env_ids is None:
        env_ids = torch.arange(env.scene.num_envs, device=env.device)
    asset = env.scene[asset_cfg.name]
    num_resets = len(env_ids)

    env_origins = env.scene.env_origins[env_ids]
    final_pos = torch.zeros((num_resets, 3), device=env.device)
    quats = torch.zeros((num_resets, 4), device=env.device)

    # 5-Sector Certified Circuit Spawner with Zero Dead Zones:
    # Sector 1: Start Line / West Runway (X = -16.201, Y = 5.500, yaw = -1.5708 [South])
    # Sector 2: Corner 1 Runway Approach (X = -16.204, Y = -4.300, yaw = -1.5708 [South, 2.0m runway into C1])
    # Sector 3: Leg 2 Post-C1 Exit & Runway (X = -12.500, Y = -7.100, yaw = 0.0000 [East, 6.5m runway into C2])
    # Sector 4: Leg 2 Intersection Approach (X = -6.000, Y = -7.118, yaw = 0.0000 [East, 0.8m before C2 turn])
    # Sector 5: Top North Straight & C4 Approach (X = -10.500, Y = 7.100, yaw = 3.14159 [West, 4.5m into C4])
    forced_sector = os.environ.get("ARCPRO_FORCE_SECTOR")
    if forced_sector is not None and forced_sector.strip() in ("1", "2", "3", "4", "5"):
        s_idx = int(forced_sector.strip())
        sector_coords = {
            1: (-16.201, 5.500, -1.5708),
            2: (-16.204, -4.300, -1.5708),
            3: (-12.500, -7.100, 0.0000),
            4: (-6.000, -7.118, 0.0000),
            5: (-10.500, 7.100, 3.14159),
        }
        cx, cy, cyaw = sector_coords[s_idx]
        base_x = torch.full((num_resets,), cx, device=env.device)
        base_y = torch.full((num_resets,), cy, device=env.device)
        base_yaw = torch.full((num_resets,), cyaw, device=env.device)
    else:
        t1, t2, t3, t4 = get_sector_thresholds()
        r = torch.rand(num_resets, device=env.device)
        base_x = torch.where(
            r < t1,
            torch.tensor(-16.201, device=env.device),
            torch.where(
                r < t2,
                torch.tensor(-16.204, device=env.device),
                torch.where(
                    r < t3,
                    torch.tensor(-12.500, device=env.device),
                    torch.where(
                        r < t4,
                        torch.tensor(-6.000, device=env.device),
                        torch.tensor(-10.500, device=env.device),
                    ),
                ),
            ),
        )

        base_y = torch.where(
            r < t1,
            torch.tensor(5.500, device=env.device),
            torch.where(
                r < t2,
                torch.tensor(-4.300, device=env.device),
                torch.where(
                    r < t3,
                    torch.tensor(-7.100, device=env.device),
                    torch.where(
                        r < t4,
                        torch.tensor(-7.118, device=env.device),
                        torch.tensor(7.100, device=env.device),
                    ),
                ),
            ),
        )

        base_yaw = torch.where(
            r < t1,
            torch.tensor(-1.5708, device=env.device),
            torch.where(
                r < t2,
                torch.tensor(-1.5708, device=env.device),
                torch.where(
                    r < t3,
                    torch.tensor(0.0000, device=env.device),
                    torch.where(
                        r < t4,
                        torch.tensor(0.0000, device=env.device),
                        torch.tensor(3.14159265, device=env.device),
                    ),
                ),
            ),
        )

    # Domain Randomization: Lateral Jitter (±1.0cm) & Heading Noise (±2 deg)
    # Prevents fixed-trajectory memorization while staying safely within 44cm lane!
    lat_jitter = (torch.rand(num_resets, device=env.device) * 2.0 - 1.0) * 0.010
    head_noise = (torch.rand(num_resets, device=env.device) * 2.0 - 1.0) * (
        2.0 * math.pi / 180.0
    )

    # Apply lateral jitter perpendicular to track heading
    # Heading is South (-1.5708 rad): lateral normal is (-sin(yaw), cos(yaw)) = (1, 0)
    spawn_x = base_x - torch.sin(base_yaw) * lat_jitter
    spawn_y = base_y + torch.cos(base_yaw) * lat_jitter
    spawn_yaw = base_yaw + head_noise

    final_pos[:, 0] = env_origins[:, 0] + spawn_x
    final_pos[:, 1] = env_origins[:, 1] + spawn_y
    final_pos[:, 2] = env_origins[:, 2] + 0.12  # Safe drop height

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
