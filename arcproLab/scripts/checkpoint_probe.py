#!/usr/bin/env python3
"""Instant CPU policy weight probe for ARCPro reinforcement learning checkpoints.

Zero GPU VRAM, zero simulation overhead (~0.1s execution).
Evaluates policy head weights under nominal straightaway and turn conditions.
"""

import argparse
import json
import os
import sys
from typing import Any, Dict

import torch


def evaluate_checkpoint(ckpt_path: str) -> Dict[str, Any]:
    """Load policy weights onto CPU and measure steering bias, turn response, and braking."""
    if not os.path.isfile(ckpt_path):
        raise FileNotFoundError(f"Checkpoint not found: {ckpt_path}")

    ckpt = torch.load(ckpt_path, map_location="cpu")
    sd = ckpt.get("policy", ckpt)

    log_std = sd["log_std_parameter"]
    std = torch.exp(log_std).numpy()
    w0 = sd["policy_head.0.weight"]
    b0 = sd["policy_head.0.bias"]
    w2 = sd["policy_head.2.weight"]
    b2 = sd["policy_head.2.bias"]

    def eval_policy(inp: torch.Tensor) -> torch.Tensor:
        h = torch.relu(torch.matmul(inp, w0.T) + b0)
        return torch.tanh(torch.matmul(h, w2.T) + b2)

    # 1. Nominal Straightaway (centered, speed=0.60, go=1.0, straight=0.0)
    # Combined vector slots: 64=turn_token, 65=go_signal, 67=speed, 69=prev_steer, 70=prev_throt
    neutral = torch.zeros(1, 76)
    neutral[0, 65] = 1.0  # go_signal
    neutral[0, 67] = 0.60  # nominal speed
    neutral[0, 69] = 0.0  # prev_steer = centered
    neutral[0, 70] = 0.60  # prev_throt = driving
    act_neutral = eval_policy(neutral).squeeze().numpy()

    # 2. Turn Token = -1.0 (Corner 2 / Left Turn Command)
    turn_cmd = neutral.clone()
    turn_cmd[0, 64] = -1.0
    act_turn = eval_policy(turn_cmd).squeeze().numpy()

    # 3. Corner Turn+Curvature (Turn Token = -1.0 + Curvature Kappa = -0.25)
    corner = turn_cmd.clone()
    corner[0, 74] = -0.25
    act_corner = eval_policy(corner).squeeze().numpy()

    bias = float(act_neutral[0])
    turn_steer = float(act_turn[0])
    delta_steer = turn_steer - bias
    pre_brake = float(act_turn[1])

    return {
        "checkpoint": os.path.basename(ckpt_path),
        "path": os.path.abspath(ckpt_path),
        "mu_neutral_steer": round(bias, 4),
        "mu_neutral_throt": round(float(act_neutral[1]), 4),
        "mu_turn_steer": round(turn_steer, 4),
        "mu_turn_throt": round(pre_brake, 4),
        "delta_turn_steer": round(delta_steer, 4),
        "mu_corner_steer": round(float(act_corner[0]), 4),
        "mu_corner_throt": round(float(act_corner[1]), 4),
        "sigma_steer": round(float(std[0]), 4),
        "sigma_throt": round(float(std[1]), 4),
        "is_neutral": abs(bias) <= 0.02,
        "is_braking_in_turn": pre_brake < -0.30,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="ARCPro Checkpoint Weight Probe")
    parser.add_argument("checkpoint", type=str, help="Path to agent_*.pt")
    parser.add_argument("--json", action="store_true", help="Output raw JSON")
    args = parser.parse_args()

    try:
        metrics = evaluate_checkpoint(args.checkpoint)
    except Exception as e:
        print(f"Error evaluating checkpoint: {e}", file=sys.stderr)
        sys.exit(1)

    if args.json:
        print(json.dumps(metrics))
        return

    b = metrics["mu_neutral_steer"]
    target_str = "✅ TARGET MET" if metrics["is_neutral"] else "⚠️ ACTIVE BIAS"
    brake_str = (
        "✅ ACTIVE BRAKING" if metrics["is_braking_in_turn"] else "❌ NO BRAKING"
    )

    print("=" * 70)
    print(f"ARCPro Policy Probe: {metrics['checkpoint']}")
    print("=" * 70)
    print(f"  Neutral Straight Steer: {b:+.4f} (Target |bias| <= 0.02) -> {target_str}")
    print(f"  Neutral Straight Throt: {metrics['mu_neutral_throt']:+.4f}")
    print(
        f"  Turn Token -1.0 Steer:  {metrics['mu_turn_steer']:+.4f} (ΔSteer = {metrics['delta_turn_steer']:+.4f})"
    )
    print(f"  Turn Token -1.0 Throt:  {metrics['mu_turn_throt']:+.4f} -> {brake_str}")
    print(
        f"  Corner Turn+Curve:      Steer={metrics['mu_corner_steer']:+.4f}, Throt={metrics['mu_corner_throt']:+.4f}"
    )
    print(
        f"  Exploration Noise:      Steer σ={metrics['sigma_steer']:.4f}, Throt σ={metrics['sigma_throt']:.4f}"
    )
    print("=" * 70)


if __name__ == "__main__":
    main()
