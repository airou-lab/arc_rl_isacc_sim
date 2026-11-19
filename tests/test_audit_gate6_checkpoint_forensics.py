#!/usr/bin/env python3
"""Gate 6 Forensic Audit: Checkpoint Forensics, Weight Norms & Rollback vs Fresh Start Decision."""

import os
import sys

import gymnasium as gym
import pytest
import torch

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ARCPRO_LAB_DIR = os.path.join(ROOT_DIR, "arcproLab")
for _p in (ROOT_DIR, ARCPRO_LAB_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from agents.skrl_models import ARCProActor, ARCProCritic  # noqa: E402

pytestmark = pytest.mark.cpu

CHECKPOINT_PATHS = [
    (
        "agent_3200000.pt",
        "logs/ppo_skrl_rollback_clean-baseline/20261109-020553/26-11-09_02-05-53-020893_TelemetryPPO/checkpoints/agent_3200000.pt"
        if not os.path.exists(
            "logs/ppo_skrl_rollback_clean-baseline/20261109-163808/26-11-09_16-38-08-480321_TelemetryPPO/checkpoints/agent_3200000.pt"
        )
        else "logs/ppo_skrl_rollback_clean-baseline/20261109-163808/26-11-09_16-38-08-480321_TelemetryPPO/checkpoints/agent_3200000.pt",
    ),
    (
        "agent_3250000.pt",
        "logs/ppo_skrl_rollback_clean-baseline/20261109-163808/26-11-09_16-38-08-480321_TelemetryPPO/checkpoints/agent_3250000.pt",
    ),
    (
        "agent_3300000.pt",
        "logs/ppo_skrl_rollback_clean-baseline/20261109-163808/26-11-09_16-38-08-480321_TelemetryPPO/checkpoints/agent_3300000.pt",
    ),
    (
        "agent_3350000.pt",
        "logs/ppo_skrl_rollback_clean-baseline/20261109-163808/26-11-09_16-38-08-480321_TelemetryPPO/checkpoints/agent_3350000.pt",
    ),
]

SLOT_NAMES = [
    "turn_token (0)",
    "dist_g / go (1)",
    "goal_dist (2)",
    "v_x speed (3)",
    "yaw_rate (4)",
    "steer_angle (5)",
    "prev_action0 (6)",
    "lat_vel (7)",
    "lat_err (8)",
    "head_err (9)",
    "lookahead_kappa (10)",
    "distance (11)",
]


def print_table(headers, rows):
    col_widths = [len(h) for h in headers]
    for row in rows:
        for i, val in enumerate(row):
            col_widths[i] = max(col_widths[i], len(str(val)))

    sep = "+-" + "-+-".join("-" * w for w in col_widths) + "-+"
    print(sep)
    header_line = (
        "| "
        + " | ".join(f"{h:<{col_widths[i]}}" for i, h in enumerate(headers))
        + " |"
    )
    print(header_line)
    print(sep)
    for row in rows:
        line = (
            "| "
            + " | ".join(f"{str(val):<{col_widths[i]}}" for i, val in enumerate(row))
            + " |"
        )
        print(line)
    print(sep)


def audit_checkpoint(name: str, path: str):
    assert os.path.isfile(path), f"Checkpoint file missing: {path}"
    ckpt = torch.load(path, map_location="cpu", weights_only=False)

    opt_state = ckpt.get("optimizer", ckpt.get("policy_optimizer", {})).get(
        "state", {}
    )
    step_param0 = None
    core_steps = []
    for k, v in opt_state.items():
        if isinstance(v, dict) and "step" in v:
            val = v["step"].item() if torch.is_tensor(v["step"]) else v["step"]
            if k == 0 or (isinstance(k, int) and k == 0):
                step_param0 = val
            else:
                core_steps.append(val)
    core_step = core_steps[0] if core_steps else step_param0

    c_dict = ckpt.get("critic", ckpt.get("value"))
    assert c_dict is not None, "Neither 'critic' nor 'value' key found in checkpoint!"
    c_w0 = c_dict["critic_head.0.weight"]
    c_norms = c_w0.norm(dim=0).tolist()

    p_dict = ckpt["policy"]
    p_w0 = p_dict["policy_head.0.weight"]
    p_telem_norms = p_w0[:, 64:76].norm(dim=0).tolist()
    p_b2 = p_dict["policy_head.2.bias"].tolist()

    log_std = p_dict.get("log_std_parameter", torch.zeros(2))
    std = torch.exp(log_std).tolist()

    return {
        "name": name,
        "path": path,
        "opt_step_param0": step_param0,
        "opt_step_core": core_step,
        "critic_norms": c_norms,
        "actor_telem_norms": p_telem_norms,
        "policy_bias": p_b2,
        "std": std,
    }


def test_gate6_checkpoint_forensics(tmp_path):
    """Run checkpoint forensics on synthetic and any available historical checkpoints."""
    obs_space = gym.spaces.Box(low=-float("inf"), high=float("inf"), shape=(25100,))
    act_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(2,))
    actor = ARCProActor(obs_space, act_space, device="cpu")
    critic = ARCProCritic(obs_space, act_space, device="cpu")

    ckpt_file = str(tmp_path / "agent_test.pt")
    torch.save(
        {
            "policy": actor.state_dict(),
            "value": critic.state_dict(),
            "policy_optimizer": {
                "state": {0: {"step": torch.tensor(100.0)}, 1: {"step": torch.tensor(100.0)}}
            },
        },
        ckpt_file,
    )
    res = audit_checkpoint("agent_test.pt", ckpt_file)
    assert len(res["critic_norms"]) == 12
    assert len(res["actor_telem_norms"]) == 12
    assert len(res["policy_bias"]) == 2
    assert res["opt_step_param0"] == 100.0

    for name, rel_path in CHECKPOINT_PATHS:
        full_path = os.path.join(ROOT_DIR, rel_path)
        if os.path.isfile(full_path):
            hist_res = audit_checkpoint(name, full_path)
            assert len(hist_res["critic_norms"]) == 12
            assert len(hist_res["actor_telem_norms"]) == 12


def main():
    results = []
    for name, rel_path in CHECKPOINT_PATHS:
        full_path = os.path.join(ROOT_DIR, rel_path)
        if os.path.isfile(full_path):
            results.append(audit_checkpoint(name, full_path))
    if results:
        headers_c = ["Feature Slot"] + [r["name"] for r in results]
        critic_table = []
        for slot_idx in range(12):
            row = [f"Slot {slot_idx:2d}: {SLOT_NAMES[slot_idx]}"]
            for r in results:
                row.append(f"{r['critic_norms'][slot_idx]:.4f}")
            critic_table.append(row)
        print_table(headers_c, critic_table)


if __name__ == "__main__":
    main()
