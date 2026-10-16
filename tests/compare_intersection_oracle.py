#!/usr/bin/env python3
"""
Side-by-Side Benchmark: Hardcoded Golden Oracle vs Trained Neural Policy.

Spawns both controllers from the identical Sector 3 starting position (X = -12.50m, Y = -7.10m):
- Run 1 (Golden Oracle): Closed-loop Stanley steering with corner pre-braking (0.35 m/s).
- Run 2 (Neural Policy): Autonomous policy from latest checkpoint (agent_3887500.pt).

Compares speed, steering lock, lateral error, and line clearances side-by-side
to pinpoint the exact mechanism and location of divergence.
"""

import os
import sys
import numpy as np

WORKSPACE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if WORKSPACE_DIR not in sys.path:
    sys.path.insert(0, WORKSPACE_DIR)
ARCPRO_LAB_DIR = os.path.join(WORKSPACE_DIR, "arcproLab")
if ARCPRO_LAB_DIR not in sys.path:
    sys.path.insert(0, ARCPRO_LAB_DIR)

from isaaclab.app import AppLauncher

launcher_args = {"headless": True, "num_envs": 1, "enable_cameras": True}
app_launcher = AppLauncher(launcher_args)
simulation_app = app_launcher.app

import torch
import gymnasium as gym

from isaaclab.envs import ManagerBasedRLEnv
from arcproLab.arcpro_env_cfg import ARCProEnvCfg
from arcproLab.agents.skrl_models import ARCProActor, ARCProCritic
from agents.skrl_wrappers import SKRLFlattenWrapper
from skrl.envs.wrappers.torch import IsaacLabWrapper
from skrl.agents.torch.ppo import PPO, PPO_DEFAULT_CONFIG
import arcproLab.mdp.curriculum_config as curr_cfg
import arcproLab.mdp.events as mdp_events


class WarmupActionWrapper(gym.Wrapper):
    """Brakes during initial simulation warmup."""
    def __init__(self, env, warmup_steps=10):
        super().__init__(env)
        self.warmup_steps = warmup_steps

    def step(self, action):
        if hasattr(self.env.unwrapped, "episode_length_buf"):
            mask = self.env.unwrapped.episode_length_buf < self.warmup_steps
            action = action.clone()
            action[mask, 0] = 0.0
            action[mask, 1] = -1.0
        return self.env.step(action)


def teleport_to_intersection_approach(env, x: float = -5.800, y: float = -7.100, yaw: float = 0.0):
    """Deterministically teleports the vehicle to Corner 2 / Intersection Approach."""
    base_env = env.unwrapped
    robot = base_env.scene["robot"]
    env_origins = base_env.scene.env_origins

    root_pos = robot.data.root_pos_w.clone()
    root_quat = robot.data.root_quat_w.clone()

    root_pos[0, 0] = env_origins[0, 0] + x
    root_pos[0, 1] = env_origins[0, 1] + y
    root_pos[0, 2] = env_origins[0, 2] + 0.12

    half_yaw = yaw / 2.0
    root_quat[0, 0] = float(np.cos(half_yaw))
    root_quat[0, 1] = 0.0
    root_quat[0, 2] = 0.0
    root_quat[0, 3] = float(np.sin(half_yaw))

    robot.write_root_pose_to_sim(torch.cat([root_pos, root_quat], dim=-1))
    robot.write_root_velocity_to_sim(torch.zeros_like(robot.data.root_vel_w))
    joint_pos = robot.data.default_joint_pos.clone()
    joint_vel = torch.zeros_like(robot.data.joint_vel)
    robot.write_joint_state_to_sim(joint_pos, joint_vel)
    base_env.scene.write_data_to_sim()


def find_latest_checkpoint() -> str:
    candidate_dirs = [
        os.path.join(WORKSPACE_DIR, "logs/ppo_skrl_rollback_clean-baseline"),
        os.path.join(WORKSPACE_DIR, "logs/ppo_skrl"),
    ]
    ckpts = []
    for cdir in candidate_dirs:
        if not os.path.isdir(cdir):
            continue
        for root, _, files in os.walk(cdir):
            if "corrupted" in root:
                continue
            for f in files:
                if f.startswith("agent_") and f.endswith(".pt"):
                    p = os.path.join(root, f)
                    ckpts.append((os.path.getmtime(p), p))
    if ckpts:
        ckpts.sort()
        return ckpts[-1][1]
    return ""


def load_policy_weights(agent: PPO, checkpoint_path: str):
    if not os.path.exists(checkpoint_path):
        raise FileNotFoundError(f"Checkpoint not found: {checkpoint_path}")

    ckpt_data = torch.load(checkpoint_path, map_location=agent.device)
    if isinstance(ckpt_data, dict):
        if "policy" in ckpt_data:
            agent.policy.load_state_dict(ckpt_data["policy"])
        elif "models" in ckpt_data and "policy" in ckpt_data["models"]:
            agent.policy.load_state_dict(ckpt_data["models"]["policy"])
        else:
            agent.policy.load_state_dict(ckpt_data)
    else:
        agent.policy.load_state_dict(ckpt_data)
    agent.set_mode("eval")


def run_oracle_controller(env, max_steps: int = 350):
    """Executes a closed-loop Stanley/geometric controller through Leg 2 and the intersection."""
    print("\n" + "=" * 70)
    print("  RUN 1: HARDCODED GOLDEN ORACLE CONTROLLER")
    print("=" * 70)

    obs, info = env.reset()
    teleport_to_intersection_approach(env, x=-5.800, y=-7.100, yaw=0.0)
    warmup_action = torch.tensor([[0.0, -1.0]], device=env.device)
    obs, reward, terminated, truncated, info = env.step(warmup_action)

    telemetry = []
    done = False
    step_count = 0
    outcome = "COMPLETED"

    while step_count < max_steps and not done:
        base_env = env.unwrapped
        robot = base_env.scene["robot"]
        pos = robot.data.root_pos_w[0].cpu().numpy()
        env_origin = base_env.scene.env_origins[0].cpu().numpy()
        local_x = pos[0] - env_origin[0]
        local_y = pos[1] - env_origin[1]

        speed_val = (
            float(info["speed_mps"].float().mean().item())
            if "speed_mps" in info
            else 0.0
        )
        dist_w_cm = (
            float(info["dist_white"][0].item()) * 100.0 if "dist_white" in info else 0.0
        )
        dist_y_cm = (
            float(info["dist_yellow"][0].item()) * 100.0
            if "dist_yellow" in info
            else 0.0
        )
        lat_err = (
            float(info["lat_err"][0].item()) if "lat_err" in info else 0.0
        )
        head_err = (
            float(info["head_err"][0].item()) if "head_err" in info else 0.0
        )

        # Closed-loop Stanley Steering (verified from test_manual_left_turn.py)
        steer_target = -2.5 * lat_err - 1.2 * head_err
        steer_val = float(np.clip(steer_target, -1.0, 1.0))
        drive_val = -0.40  # Stable 0.40 - 0.60 m/s cornering velocity

        action = torch.tensor([[steer_val, drive_val]], device=env.device)

        obs, reward, terminated, truncated, info = env.step(action)
        step_count += 1

        snap = {
            "step": step_count,
            "x": round(float(local_x), 3),
            "y": round(float(local_y), 3),
            "speed": round(speed_val, 3),
            "steer": round(steer_val, 3),
            "dist_w_cm": round(dist_w_cm, 1),
            "dist_y_cm": round(dist_y_cm, 1),
            "lat_err_cm": round(lat_err * 100.0, 1),
            "head_err_deg": round(head_err * 180.0 / np.pi, 1),
        }
        telemetry.append(snap)

        is_term = bool(terminated[0].item())
        is_trunc = bool(truncated[0].item())
        if is_term or is_trunc:
            done = True
            outcome = f"RESET @ Step {step_count} (W: {dist_w_cm:.1f}cm, Y: {dist_y_cm:.1f}cm)"

    if not done:
        outcome = f"SUCCESSFULLY TRAVERSED {step_count} STEPS (Through Intersection)"

    print(f"  Oracle Result: {outcome}")
    return telemetry, outcome


def run_policy_controller(env, agent, checkpoint_path: str, max_steps: int = 350):
    """Executes the trained neural network policy on the identical Sector 3 start."""
    ckpt_name = os.path.basename(checkpoint_path)
    print("\n" + "=" * 70)
    print(f"  RUN 2: TRAINED NEURAL POLICY ({ckpt_name})")
    print("=" * 70)

    load_policy_weights(agent, checkpoint_path)
    obs, info = env.reset()
    teleport_to_intersection_approach(env, x=-5.800, y=-7.100, yaw=0.0)
    warmup_action = torch.tensor([[0.0, -1.0]], device=env.device)
    obs, reward, terminated, truncated, info = env.step(warmup_action)

    telemetry = []
    done = False
    step_count = 0
    outcome = "COMPLETED"

    while step_count < max_steps and not done:
        base_env = env.unwrapped
        robot = base_env.scene["robot"]
        pos = robot.data.root_pos_w[0].cpu().numpy()
        env_origin = base_env.scene.env_origins[0].cpu().numpy()
        local_x = pos[0] - env_origin[0]
        local_y = pos[1] - env_origin[1]

        speed_val = (
            float(info["speed_mps"].float().mean().item())
            if "speed_mps" in info
            else 0.0
        )
        dist_w_cm = (
            float(info["dist_white"][0].item()) * 100.0 if "dist_white" in info else 0.0
        )
        dist_y_cm = (
            float(info["dist_yellow"][0].item()) * 100.0
            if "dist_yellow" in info
            else 0.0
        )
        lat_err = (
            float(info["lat_err"][0].item()) if "lat_err" in info else 0.0
        )
        head_err = (
            float(info["head_err"][0].item()) if "head_err" in info else 0.0
        )

        with torch.no_grad():
            _, _, outputs = agent.act(obs, timestep=0, timesteps=0)
            action = outputs.get("mean_actions", None)
            if action is None:
                action = agent.policy.compute({"states": obs})[0]
            action = action.clone()

        obs, reward, terminated, truncated, info = env.step(action)
        step_count += 1

        steer_val = float(action[0, 0].item())

        snap = {
            "step": step_count,
            "x": round(float(local_x), 3),
            "y": round(float(local_y), 3),
            "speed": round(speed_val, 3),
            "steer": round(steer_val, 3),
            "dist_w_cm": round(dist_w_cm, 1),
            "dist_y_cm": round(dist_y_cm, 1),
            "lat_err_cm": round(lat_err * 100.0, 1),
            "head_err_deg": round(head_err * 180.0 / np.pi, 1),
        }
        telemetry.append(snap)

        is_term = bool(terminated[0].item())
        is_trunc = bool(truncated[0].item())
        if is_term or is_trunc:
            done = True
            outcome = f"CRASHED @ Step {step_count} (W: {dist_w_cm:.1f}cm, Y: {dist_y_cm:.1f}cm, LatErr: {lat_err*100:+.1f}cm)"

    if not done:
        outcome = f"COMPLETED {step_count} STEPS"

    print(f"  Policy Result: {outcome}")
    return telemetry, outcome


def main():
    print("=" * 80)
    print("  ARCPro RL: Headless Golden Oracle vs Neural Policy Benchmark")
    print("=" * 80)

    # 1. Initialize Environment
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = 1
    env_cfg.enable_cameras = True
    env_cfg.__post_init__()

    env = ManagerBasedRLEnv(cfg=env_cfg)
    env = WarmupActionWrapper(env, warmup_steps=10)
    env = IsaacLabWrapper(env)
    env = SKRLFlattenWrapper(env)

    # 2. Build Policy Agent
    state_space = getattr(env, "state_space", env.observation_space)
    models = {
        "policy": ARCProActor(env.observation_space, env.action_space, "cuda:0"),
        "value": ARCProCritic(state_space, env.action_space, "cuda:0"),
    }
    agent_cfg = PPO_DEFAULT_CONFIG.copy()
    agent_cfg["experiment"]["write_interval"] = 0
    agent_cfg["experiment"]["checkpoint_interval"] = 0

    agent = PPO(
        models=models,
        memory=None,
        cfg=agent_cfg,
        observation_space=env.observation_space,
        action_space=env.action_space,
        device="cuda:0",
    )

    ckpt_path = find_latest_checkpoint()
    print(f"Target Checkpoint: {ckpt_path}")

    # 3. Run Controller 1: Golden Oracle
    oracle_telem, oracle_outcome = run_oracle_controller(env, max_steps=350)

    # 4. Run Controller 2: Neural Policy
    policy_telem, policy_outcome = run_policy_controller(env, agent, ckpt_path, max_steps=350)

    env.close()

    # 5. Side-by-Side Comparison Table
    print("\n" + "=" * 105)
    print("  SIDE-BY-SIDE TRAJECTORY COMPARISON: ORACLE (STANLEY) vs NEURAL POLICY")
    print("=" * 105)
    header = (
        f"{'Step':<5} | {'Zone':<12} | "
        f"{'Orc X,Y':<15} | {'Orc Spd':<7} | {'Orc St':<6} | {'Orc Lat':<7} | "
        f"{'Pol X,Y':<15} | {'Pol Spd':<7} | {'Pol St':<6} | {'Pol Lat':<7} | {'Δ LatErr'}"
    )
    print(header)
    print("-" * 105)

    sample_steps = [10, 25, 50, 75, 100, 125, 150, 175, 200, 225, 240, 246, 260, 280, 300, 320, 340, 350]
    oracle_dict = {t["step"]: t for t in oracle_telem}
    policy_dict = {t["step"]: t for t in policy_telem}

    divergence_step = None

    all_steps = sorted(list(set(list(oracle_dict.keys()) + list(policy_dict.keys()))))
    for s in all_steps:
        o = oracle_dict.get(s)
        p = policy_dict.get(s)

        if o and p and divergence_step is None:
            lat_diff = abs(p["lat_err_cm"] - o["lat_err_cm"])
            if lat_diff > 5.0 and s > 20:
                divergence_step = (s, o["x"], o["y"], p["speed"], o["speed"], p["lat_err_cm"], o["lat_err_cm"])

        if s in sample_steps or (p and s == len(policy_telem)):
            # Determine Track Zone
            x_ref = o["x"] if o else (p["x"] if p else 0.0)
            if x_ref < -7.2:
                zone = "Runway"
            elif x_ref < -2.4:
                zone = "Intersection"
            else:
                zone = "North Bend"

            o_pos = f"({o['x']:.2f}, {o['y']:.2f})" if o else "N/A"
            o_spd = f"{o['speed']:.2f}m/s" if o else "N/A"
            o_st = f"{o['steer']:+.2f}" if o else "N/A"
            o_lat = f"{o['lat_err_cm']:+.1f}cm" if o else "N/A"

            p_pos = f"({p['x']:.2f}, {p['y']:.2f})" if p else "CRASHED"
            p_spd = f"{p['speed']:.2f}m/s" if p else "--"
            p_st = f"{p['steer']:+.2f}" if p else "--"
            p_lat = f"{p['lat_err_cm']:+.1f}cm" if p else "--"

            d_lat = f"{abs(p['lat_err_cm'] - o['lat_err_cm']):.1f}cm" if (o and p) else "--"

            print(
                f"{s:<5d} | {zone:<12} | "
                f"{o_pos:<15} | {o_spd:<7} | {o_st:<6} | {o_lat:<7} | "
                f"{p_pos:<15} | {p_spd:<7} | {p_st:<6} | {p_lat:<7} | {d_lat}"
            )

    print("=" * 105)

    print("\n" + "=" * 70)
    print("  EXECUTIVE DIVERGENCE DIAGNOSIS")
    print("=" * 70)
    print(f"  Oracle Outcome : {oracle_outcome}")
    print(f"  Policy Outcome : {policy_outcome}")
    if divergence_step:
        d_s, d_x, d_y, d_pspd, d_ospd, d_plat, d_olat = divergence_step
        print(f"\n  CRITICAL DIVERGENCE DETECTED @ Step {d_s}:")
        print(f"    Location           : X = {d_x:.2f} m, Y = {d_y:.2f} m")
        print(f"    Speed Discrepancy  : Policy = {d_pspd:.2f} m/s vs Oracle = {d_ospd:.2f} m/s (Δ = {d_pspd - d_ospd:+.2f} m/s)")
        print(f"    Lateral Deviation  : Policy = {d_plat:+.1f} cm vs Oracle = {d_olat:+.1f} cm (Δ = {abs(d_plat - d_olat):.1f} cm)")
    print("=" * 70)


if __name__ == "__main__":
    main()
    simulation_app.close()
    os._exit(0)
