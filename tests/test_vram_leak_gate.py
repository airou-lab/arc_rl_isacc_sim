#!/usr/bin/env python3
"""Adversarial 200-Step VRAM Memory Stability Gate (Isaac Sim GPU test)."""

import argparse
import gc
import os
import sys

import pytest
import torch

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ARCPRO_LAB_DIR = os.path.join(ROOT_DIR, "arcproLab")
for _p in (ROOT_DIR, ARCPRO_LAB_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

pytestmark = pytest.mark.isaac


def main():
    from isaaclab.app import AppLauncher

    parser = argparse.ArgumentParser(
        description="Adversarial 200-Step VRAM Memory Stability Gate"
    )
    parser.add_argument("--num_envs", type=int, default=20)
    AppLauncher.add_app_launcher_args(parser)
    args_cli = parser.parse_args()

    app_launcher = AppLauncher(args_cli)
    simulation_app = app_launcher.app

    from agents.skrl_wrappers import SKRLFlattenWrapper
    from arcpro_env_cfg import ARCProEnvCfg
    from isaaclab.envs import ManagerBasedRLEnv
    from skrl.envs.wrappers.torch import IsaacLabWrapper

    print("=" * 70)
    print("ADVERSARIAL VRAM LEAK VERIFICATION GATE (200 STEPS, 20 RGB ENVS)")
    print("=" * 70)

    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.enable_cameras = True
    env_cfg.__post_init__()

    base_env = ManagerBasedRLEnv(cfg=env_cfg)
    isaac_env = IsaacLabWrapper(base_env)
    wrapped_env = SKRLFlattenWrapper(isaac_env)

    obs, info = wrapped_env.reset()
    torch.cuda.synchronize()

    for _ in range(10):
        actions = torch.zeros((args_cli.num_envs, 2), device="cuda:0")
        with torch.no_grad():
            obs, rew, term, trunc, info = wrapped_env.step(actions)

    gc.collect()
    torch.cuda.empty_cache()
    torch.cuda.synchronize()

    start_alloc_mb = torch.cuda.memory_allocated() / (1024 * 1024)
    start_reserved_mb = torch.cuda.memory_reserved() / (1024 * 1024)
    print(
        f"[GATE] Stabilized Post-Warmup Baseline: Allocated={start_alloc_mb:.2f} MB, Reserved={start_reserved_mb:.2f} MB"
    )

    num_test_steps = 200
    for step in range(1, num_test_steps + 1):
        actions = torch.zeros((args_cli.num_envs, 2), device="cuda:0")
        with torch.no_grad():
            obs, rew, term, trunc, info = wrapped_env.step(actions)

        if step % 50 == 0:
            torch.cuda.synchronize()
            cur_alloc_mb = torch.cuda.memory_allocated() / (1024 * 1024)
            delta_mb = cur_alloc_mb - start_alloc_mb
            print(
                f"[GATE] Step {step:3d}/{num_test_steps}: Allocated={cur_alloc_mb:.2f} MB | Delta={delta_mb:+.2f} MB"
            )

    torch.cuda.synchronize()
    final_alloc_mb = torch.cuda.memory_allocated() / (1024 * 1024)
    final_delta_mb = final_alloc_mb - start_alloc_mb

    wrapped_env.close()
    simulation_app.close()

    MAX_ALLOWED_DELTA_MB = 10.0
    if final_delta_mb > MAX_ALLOWED_DELTA_MB:
        sys.exit(1)
    sys.exit(0)


@pytest.mark.isaac_sim
def test_vram_leak_gate():
    pytest.importorskip("isaaclab")
    main()


if __name__ == "__main__":
    main()
