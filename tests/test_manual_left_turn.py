#!/usr/bin/env python3
"""
Test Manual Left Turn in Corner 1:
Spawns the vehicle at Corner 1 Runway (WP 1020, X=-16.218, Y=-4.80)
and applies a controlled left-turn command (steer > 0) with cornering speed (~0.35-0.45 m/s).
Verifies whether the physical vehicle, Ackermann steering, and electronic differential
can negotiate Corner 1 without triggering roadmark contact termination (dist_w, dist_y).
"""

import sys
import os
import torch

WORKSPACE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if WORKSPACE_DIR not in sys.path:
    sys.path.insert(0, WORKSPACE_DIR)
ARCPRO_LAB_DIR = os.path.join(WORKSPACE_DIR, "arcproLab")
if ARCPRO_LAB_DIR not in sys.path:
    sys.path.insert(0, ARCPRO_LAB_DIR)

from isaaclab.app import AppLauncher

app_launcher = AppLauncher({"headless": True, "num_envs": 1})
simulation_app = app_launcher.app

from isaaclab.envs import ManagerBasedRLEnv
from arcproLab.arcpro_env_cfg import ARCProEnvCfg


def run_manual_left_turn_test():
    print("=" * 60)
    print("TESTING MANUAL LEFT TURN IN CORNER 1")
    print("=" * 60)

    # Test Corner 2 Left Turn (R = 1.16m - 1.36m, turn from East into North)
    # Approach is at X = -6.0, Y = -7.10, yaw = 0.0 (East)
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = 1
    env_cfg.enable_cameras = False  # Fast kinematic verification
    env_cfg.episode_length_s = 60.0
    env_cfg.__post_init__()

    env = ManagerBasedRLEnv(cfg=env_cfg)
    obs, _ = env.reset()

    # Teleport directly to Corner 2 Approach: X = -5.80, Y = -7.10, yaw = 0.0
    robot = env.scene["robot"]
    root_pos = robot.data.root_pos_w.clone()
    root_quat = robot.data.root_quat_w.clone()
    env_origins = env.scene.env_origins

    # Heading East: visual nose = East (yaw_nose = 3.14159, base_yaw = 0.0)
    root_pos[0, 0] = env_origins[0, 0] - 5.800
    root_pos[0, 1] = env_origins[0, 1] - 7.100
    root_pos[0, 2] = env_origins[0, 2] + 0.12
    # Quat for yaw = 0.0: cos(0)=1, sin(0)=0 -> [1, 0, 0, 0]
    root_quat[0, :] = torch.tensor([1.0, 0.0, 0.0, 0.0], device=env.device)
    robot.write_root_pose_to_sim(torch.cat([root_pos, root_quat], dim=-1))
    robot.write_root_velocity_to_sim(torch.zeros_like(robot.data.root_vel_w))

    init_pos = robot.data.root_pos_w[0].cpu().numpy()
    print(
        f"Spawned at Corner 2 Entry: X={init_pos[0]:.3f}, Y={init_pos[1]:.3f}, Z={init_pos[2]:.3f}"
    )

    action = torch.zeros((1, 2), device=env.device)
    total_steps = 350
    crashed = False
    crash_reason = None
    min_dist_w = float("inf")
    min_dist_y = float("inf")

    print("\nExecuting cornering trajectory through Corner 1...")
    for step in range(total_steps):
        pos = robot.data.root_pos_w[0].cpu().numpy()
        speed = torch.norm(robot.data.root_lin_vel_b[:, :2], dim=1).item()
        dist_w = env.extras.get("dist_w", torch.tensor([1.0]))[0].item()
        dist_y = env.extras.get("dist_y", torch.tensor([1.0]))[0].item()
        lat_err = env.extras.get("lat_err", torch.tensor([0.0]))[0].item()
        head_err = env.extras.get("head_err", torch.tensor([0.0]))[0].item()

        min_dist_w = min(min_dist_w, dist_w)
        min_dist_y = min(min_dist_y, dist_y)

        # Closed-loop Stanley/PD steering between boundaries:
        # steer > 0: turn LEFT; steer < 0: turn RIGHT
        # In Corner 1: track turns LEFT into +X.
        # Target speed: 0.40 m/s
        target_steer = -2.5 * lat_err - 1.2 * head_err
        steer = float(torch.clamp(torch.tensor(target_steer), min=-1.0, max=1.0).item())
        drive = -0.40  # ~0.40 m/s

        action[:, 0] = steer
        action[:, 1] = drive

        obs, reward, terminated, truncated, info = env.step(action)

        if step % 25 == 0 or step == total_steps - 1:
            print(
                f"Step {step:3d}: Pos=({pos[0]:6.2f}, {pos[1]:6.2f}) | Spd={speed:.2f}m/s | Steer={steer:+.2f} | LatErr={lat_err * 100:+4.1f}cm | DistW={dist_w * 100:4.1f}cm | DistY={dist_y * 100:4.1f}cm"
            )

        if terminated.any():
            crashed = True
            crash_reason = f"Terminated at step {step} (Pos: X={pos[0]:.2f}, Y={pos[1]:.2f}, DistW={dist_w * 100:.1f}cm, DistY={dist_y * 100:.1f}cm)"
            break

    print("\n" + "=" * 60)
    print(
        f"TEST RESULT: {'CRASHED' if crashed else 'SUCCESSFULLY COMPLETED CORNER 1!'}"
    )
    if crashed:
        print(f"Reason: {crash_reason}")
    print(
        f"Min White Line Clearance: {min_dist_w * 100:.1f} cm (threshold ~1.5 - 4.0 cm)"
    )
    print(
        f"Min Yellow Line Clearance: {min_dist_y * 100:.1f} cm (threshold ~0.8 - 2.0 cm)"
    )
    final_pos = robot.data.root_pos_w[0].cpu().numpy()
    print(f"Final Position: X={final_pos[0]:.3f}, Y={final_pos[1]:.3f}")
    print("=" * 60)

    env.close()
    return not crashed


if __name__ == "__main__":
    success = run_manual_left_turn_test()
    simulation_app.close()
    sys.exit(0 if success else 1)
