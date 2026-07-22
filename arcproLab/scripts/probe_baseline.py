# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
"""
Phase 0 baseline harness for docs/off-road-debug/.

Read-only measurement -- changes NO behavior. Captures the "before" picture
that every later fix gets validated against. Single Isaac Sim boot,
num_envs=1, headless, no camera.

Part A  Observation integrity: dumps all 12 telemetry slots while driving
        forward, so corrupted/dead/noise slots are visible as data rather
        than as a code-reading argument.
Part B  Kinematics expected-vs-real: for a sweep of steering commands,
        compares analytically-expected Ackermann joint angles against the
        action term's processed_actions AND against what the articulation
        actually reports -- isolating "wrong command" from "command fine,
        actuator not tracking". (This is the comparison Aaron originally
        proposed.)
Part C  Straight-line drift: 0.0 steering, constant throttle. Measures
        lateral deviation from the initial heading, and reports which way
        the robot actually travels vs its local axes.
Part D  Termination cause: per-DoneTerm attribution via
        TerminationManager.get_term(), so we can tell WHICH rule ended an
        episode instead of guessing.
"""
import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Phase 0 baseline probe (read-only).")
parser.add_argument("--settle_steps", type=int, default=60, help="Steps to let the robot drop/settle before measuring.")
parser.add_argument("--drive_steps", type=int, default=400, help="Steps to drive forward in Parts A/C/D.")
parser.add_argument("--throttle", type=float, default=0.5, help="Constant throttle for Parts A/C/D.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.headless = True
args_cli.enable_cameras = False

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import os
import sys
import math
import torch

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ARCPRO_LAB_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
if ARCPRO_LAB_DIR not in sys.path:
    sys.path.insert(0, ARCPRO_LAB_DIR)

from arcpro_env_cfg import ARCProEnvCfg
from isaaclab.envs import ManagerBasedRLEnv

SLOT_NAMES = [
    "0 turn_token", "1 go_signal", "2 goal_dist", "3 speed",
    "4 yaw_rate", "5 act_steer", "6 act_throttle", "7 act_brake",
    "8 lat_err(masked)", "9 head_err(masked)", "10 kappa", "11 distance",
]


def yaw_from_quat(q):
    return math.atan2(2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 * (q[2] ** 2 + q[3] ** 2))


def expected_ackermann(action_val, wheelbase=0.33, track_width=0.28, max_steer=0.5):
    """Analytic replica of AckermannSteeringAction.process_actions()."""
    half_track = track_width / 2.0
    angle = action_val * max_steer
    if abs(angle) < 1e-4:
        return 0.0, 0.0
    turn_radius = wheelbase / math.tan(abs(angle) + 1e-6)
    inner = math.atan(wheelbase / (turn_radius - half_track))
    outer = math.atan(wheelbase / (turn_radius + half_track))
    if angle > 0:
        return outer, inner
    return -inner, -outer


def make_action(env, steer, throttle, brake=0.0):
    return torch.tensor([[steer, throttle, brake]], device=env.device, dtype=torch.float32)


def settle(env, steps):
    for _ in range(steps):
        env.step(make_action(env, 0.0, 0.0))


def main():
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = 1
    env_cfg.enable_cameras = False
    env_cfg.observations.visual = None
    env_cfg.scene.tiled_camera = None
    env_cfg.__post_init__()

    env = ManagerBasedRLEnv(cfg=env_cfg)
    robot = env.scene["robot"]

    control_dt = env_cfg.sim.dt * env_cfg.decimation
    print(f"\n[CONFIG] sim.dt={env_cfg.sim.dt} decimation={env_cfg.decimation} "
          f"-> control_dt={control_dt:.4f}s ({1.0/control_dt:.1f} Hz)", flush=True)
    print(f"[CONFIG] episode_length_s={env_cfg.episode_length_s} "
          f"-> {int(env_cfg.episode_length_s/control_dt)} control steps/episode", flush=True)
    print(f"[CONFIG] termination terms: {env.termination_manager.active_terms}", flush=True)

    # ---------------- Part B: kinematics expected vs real ----------------
    # Done first: needs a settled, stationary robot and clean joint reads.
    env.reset()
    settle(env, args_cli.settle_steps)

    steer_joint_ids, steer_joint_names = robot.find_joints(["Joint_Steer_L", "Joint_Steer_R"])
    drive_joint_ids, drive_joint_names = robot.find_joints(
        ["Joint_Drive_RL", "Joint_Drive_RR", "Joint_Drive_FL", "Joint_Drive_FR"])
    print(f"\n{'='*78}\nPART B: STEERING KINEMATICS -- EXPECTED vs REAL\n{'='*78}", flush=True)
    print(f"steer joints {steer_joint_names} ids={steer_joint_ids}", flush=True)
    print(f"{'cmd':>6} | {'exp_L':>8} {'exp_R':>8} | {'proc_L':>8} {'proc_R':>8} | {'real_L':>8} {'real_R':>8} | {'errL':>7} {'errR':>7}", flush=True)

    steer_term = env.action_manager.get_term("steering")
    for cmd in (-1.0, -0.5, 0.0, 0.5, 1.0):
        # hold the command long enough for the position-controlled joint to converge
        for _ in range(80):
            env.step(make_action(env, cmd, 0.0))
        exp_l, exp_r = expected_ackermann(cmd)
        proc = steer_term.processed_actions[0]
        real = robot.data.joint_pos[0, steer_joint_ids]
        err_l = real[0].item() - exp_l
        err_r = real[1].item() - exp_r
        print(f"{cmd:6.2f} | {exp_l:8.4f} {exp_r:8.4f} | {proc[0].item():8.4f} {proc[1].item():8.4f} | "
              f"{real[0].item():8.4f} {real[1].item():8.4f} | {err_l:7.4f} {err_r:7.4f}", flush=True)

    # drive command check
    drive_term = env.action_manager.get_term("drive")
    for thr in (0.25, 0.5, 1.0):
        for _ in range(40):
            env.step(make_action(env, 0.0, thr))
        proc_v = drive_term.processed_actions[0, 0].item()
        real_v = robot.data.joint_vel[0, drive_joint_ids].mean().item()
        print(f"[DRIVE] throttle={thr:.2f} -> commanded_joint_vel={proc_v:8.3f} rad/s | "
              f"actual_mean_joint_vel={real_v:8.3f} rad/s", flush=True)

    # ---------------- Parts A / C / D: forward rollout ----------------
    obs_dict, _ = env.reset()
    settle(env, args_cli.settle_steps)

    start_pos = robot.data.root_pos_w[0, :2].clone()
    start_quat = robot.data.root_quat_w[0].tolist()
    start_yaw = yaw_from_quat(start_quat)
    # local -X and +X expressed in world, to resolve which is "forward" empirically
    fwd_negX = torch.tensor([-math.cos(start_yaw), -math.sin(start_yaw)], device=env.device)
    lat_axis = torch.tensor([-math.sin(start_yaw), math.cos(start_yaw)], device=env.device)

    print(f"\n{'='*78}\nPARTS A/C/D: FORWARD ROLLOUT (throttle={args_cli.throttle}, steer=0.0)\n{'='*78}", flush=True)
    print(f"start pos={start_pos.tolist()} yaw={math.degrees(start_yaw):.1f}deg", flush=True)
    print(f"\n{'step':>5} | {'along':>7} {'lateral':>8} | {'speed':>6} | {'slot10 kappa':>13} {'slot11 dist':>12} | {'slot0':>6}", flush=True)

    term_counts = {name: 0 for name in env.termination_manager.active_terms}
    n_term = 0
    obs_min = [float("inf")] * 12
    obs_max = [float("-inf")] * 12
    nonfinite = [0] * 12

    for step in range(args_cli.drive_steps):
        obs_dict, rew, terminated, truncated, info = env.step(make_action(env, 0.0, args_cli.throttle))
        vec = obs_dict["policy"][0]

        for i in range(12):
            v = vec[i].item()
            if not math.isfinite(v):
                nonfinite[i] += 1
                continue
            obs_min[i] = min(obs_min[i], v)
            obs_max[i] = max(obs_max[i], v)

        if terminated[0] or truncated[0]:
            n_term += 1
            for name in env.termination_manager.active_terms:
                if env.termination_manager.get_term(name)[0].item():
                    term_counts[name] += 1
            # after an auto-reset the frame of reference is stale; re-baseline
            settle(env, 5)
            start_pos = robot.data.root_pos_w[0, :2].clone()

        if step % 50 == 0:
            d = robot.data.root_pos_w[0, :2] - start_pos
            along = torch.dot(d, fwd_negX).item()
            lateral = torch.dot(d, lat_axis).item()
            speed = torch.norm(robot.data.root_lin_vel_b[0, :2]).item()
            print(f"{step:5d} | {along:7.3f} {lateral:8.4f} | {speed:6.3f} | "
                  f"{vec[10].item():13.3f} {vec[11].item():12.3f} | {vec[0].item():6.1f}", flush=True)

    # ---------------- summaries ----------------
    print(f"\n{'='*78}\nPART A: OBSERVATION SLOT INTEGRITY (over {args_cli.drive_steps} steps)\n{'='*78}", flush=True)
    print(f"{'slot':<20} {'min':>12} {'max':>12} {'range':>12}  {'nonfinite':>9}", flush=True)
    for i, name in enumerate(SLOT_NAMES):
        rng = obs_max[i] - obs_min[i] if math.isfinite(obs_min[i]) else float("nan")
        flag = ""
        if nonfinite[i]:
            flag = "  <-- NON-FINITE"
        elif rng == 0.0:
            flag = "  <-- CONSTANT (dead slot)"
        print(f"{name:<20} {obs_min[i]:12.4f} {obs_max[i]:12.4f} {rng:12.4f}  {nonfinite[i]:9d}{flag}", flush=True)

    d = robot.data.root_pos_w[0, :2] - start_pos
    print(f"\n{'='*78}\nPART C: STRAIGHT-LINE DRIFT\n{'='*78}", flush=True)
    print(f"displacement along local -X : {torch.dot(d, fwd_negX).item():.4f} m", flush=True)
    print(f"displacement lateral        : {torch.dot(d, lat_axis).item():.4f} m", flush=True)
    print("  (positive 'along' => local -X is forward; negative => local +X is forward)", flush=True)

    print(f"\n{'='*78}\nPART D: TERMINATION CAUSES\n{'='*78}", flush=True)
    print(f"total episode ends in {args_cli.drive_steps} steps: {n_term}", flush=True)
    for name, c in sorted(term_counts.items(), key=lambda kv: -kv[1]):
        print(f"  {name:<24} {c}", flush=True)

    # ---------------- Part E: Phase 1 fix assertions ----------------
    # Falsifiable checks for the observation-integrity fixes. These compare
    # slot 11 against ground-truth displacement measured from the simulator.
    print(f"\n{'='*78}\nPART E: PHASE 1 FIX VALIDATION\n{'='*78}", flush=True)
    passed = True

    along = torch.dot(d, fwd_negX).item()
    slot11_delta = obs_max[11] - obs_min[11]

    # 1. SIGN: distance must increase while driving forward.
    sign_ok = obs_max[11] > obs_min[11] and obs_max[11] > 0
    print(f"[{'PASS' if sign_ok else 'FAIL'}] slot11 sign: min={obs_min[11]:.3f} max={obs_max[11]:.3f} "
          f"(must rise and end positive while driving forward)", flush=True)
    passed &= sign_ok

    # 2. SCALE: accumulated distance must match true displacement.
    # Compared against total path travelled (`along`), tolerance 15% for
    # wheel slip / the pre-measurement settle window.
    # Only valid when no reset occurred: `distance` zeroes on reset, so
    # min/max spanning a reset would make this comparison meaningless.
    if n_term > 0:
        print(f"[SKIP] slot11 scale: {n_term} reset(s) occurred, so min/max span a "
              f"distance reset and the ratio would be meaningless.", flush=True)
    elif along > 0.1:
        ratio = slot11_delta / along
        scale_ok = 0.85 <= ratio <= 1.15
        print(f"[{'PASS' if scale_ok else 'FAIL'}] slot11 scale: accumulated={slot11_delta:.3f} "
              f"true_displacement={along:.3f} ratio={ratio:.3f} (want ~1.0, was 2.505 pre-fix)", flush=True)
        passed &= scale_ok

    # 3. KAPPA GUARD: no value may exceed the physical bound.
    from mdp.track_manager import MAX_PATH_CURVATURE
    kappa_ok = abs(obs_min[10]) <= MAX_PATH_CURVATURE + 1e-6 and abs(obs_max[10]) <= MAX_PATH_CURVATURE + 1e-6
    print(f"[{'PASS' if kappa_ok else 'FAIL'}] slot10 kappa within +/-{MAX_PATH_CURVATURE}: "
          f"min={obs_min[10]:.3f} max={obs_max[10]:.3f}", flush=True)
    passed &= kappa_ok

    # 4. No non-finite values anywhere in the observation vector.
    finite_ok = sum(nonfinite) == 0
    print(f"[{'PASS' if finite_ok else 'FAIL'}] all slots finite: {sum(nonfinite)} non-finite readings", flush=True)
    passed &= finite_ok

    # 5. turn_token across episodes -- the Phase 0 open question.
    print(f"\n[INFO] slot0 turn_token observed values this run: "
          f"min={obs_min[0]:.1f} max={obs_max[0]:.1f} over {n_term} episode reset(s)", flush=True)
    if n_term == 0:
        print("[INFO] zero resets occurred, so per-episode turn_token randomization "
              "remains UNPROVEN (same caveat as Phase 0).", flush=True)

    print(f"\n{'[PASS] Phase 1 observation-integrity checks all passed.' if passed else '[FAIL] One or more Phase 1 checks failed -- see above.'}", flush=True)

    env.close()
    simulation_app.close()


if __name__ == "__main__":
    main()
