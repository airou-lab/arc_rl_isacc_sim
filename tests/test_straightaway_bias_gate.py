#!/usr/bin/env python3
"""Deterministic Straightaway Bias Quality Gate for ARCPro Policy Checkpoints.

Verifies:
1. Neutral Straightaway Steer on real synthesized road images: |steer| <= 0.05 rad.
2. Cornering Authority: Turn Token = -1.0 produces steer >= +0.30 rad.
3. Masked telemetry columns [69, 70, 72, 73, 75] have zero influence on probed bias.
4. Zero GPU VRAM, zero Isaac Sim overhead (~0.05s execution on CPU).
"""

import argparse
import glob
import os
import sys
from typing import Any, Dict, Optional, Tuple

import gymnasium as gym
import pytest
import torch

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ARCPRO_LAB_DIR = os.path.join(ROOT_DIR, "arcproLab")
SCRIPTS_DIR = os.path.join(ARCPRO_LAB_DIR, "scripts")
for _p in (ROOT_DIR, ARCPRO_LAB_DIR, SCRIPTS_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from agents.skrl_models import ARCProActor  # noqa: E402
from checkpoint_probe import evaluate_checkpoint  # noqa: E402
from debias_checkpoint import debias_checkpoint  # noqa: E402

pytestmark = pytest.mark.cpu


def find_latest_checkpoint(root_dir: Optional[str] = None) -> str:
    """Find the most recent checkpoint in logs/."""
    if root_dir is None:
        root_dir = ROOT_DIR
    search_dir = os.path.join(root_dir, "logs")
    candidates = glob.glob(os.path.join(search_dir, "**/agent_*.pt"), recursive=True)
    candidates = [c for c in candidates if "corrupted" not in c]
    if not candidates:
        raise FileNotFoundError(f"No valid checkpoints found under {search_dir}")
    return max(candidates, key=os.path.getmtime)


def evaluate_policy_cpu(
    ckpt_path: str,
) -> Tuple[Dict[str, float], Dict[str, Any]]:
    """Load policy weights onto CPU and evaluate deterministic response on synthesized road images."""
    if not os.path.isfile(ckpt_path):
        raise FileNotFoundError(f"Checkpoint file does not exist: {ckpt_path}")

    metrics = evaluate_checkpoint(ckpt_path)
    ckpt = torch.load(ckpt_path, map_location="cpu", weights_only=False)
    sd = (
        ckpt.get("policy", ckpt.get("models", {}).get("policy", ckpt))
        if isinstance(ckpt, dict)
        else ckpt
    )
    w0 = sd["policy_head.0.weight"]
    b0 = sd["policy_head.0.bias"]
    w2 = sd["policy_head.2.weight"]
    b2 = sd["policy_head.2.bias"]
    tensors = {
        "w0": w0,
        "b0": b0,
        "w2": w2,
        "b2": b2,
        "std": [metrics["sigma_steer"], metrics["sigma_throt"]],
    }
    return metrics, tensors


def run_bias_gate_check(
    ckpt_path: str,
    max_steer_bias: float = 0.05,
    min_turn_steer: float = 0.30,
    strict: bool = True,
) -> Tuple[bool, Dict[str, Any], str]:
    """Execute gate check against required tolerances."""
    metrics, _ = evaluate_policy_cpu(ckpt_path)

    neutral_steer = metrics["mu_neutral_steer"]
    turn_steer = metrics["mu_turn_steer"]
    delta_steer = metrics["delta_turn_steer"]
    turn_throt = metrics["mu_turn_throt"]
    sigma_steer = metrics["sigma_steer"]

    neutral_pass = abs(neutral_steer) <= max_steer_bias
    turn_pass = turn_steer >= min_turn_steer
    passed = neutral_pass and turn_pass

    lines = [
        "=" * 76,
        f"ARCPro Deterministic Straightaway Bias Gate: {os.path.basename(ckpt_path)}",
        "=" * 76,
        f"  Path: {ckpt_path}",
        f"  [1] Straightaway Neutral Steer : {neutral_steer:+.4f} rad (Limit: |steer| <= {max_steer_bias:.4f}) -> {'PASS' if neutral_pass else 'FAIL'}",
        f"  [2] Turn Token -1.0 Steer      : {turn_steer:+.4f} rad (Limit: steer >= +{min_turn_steer:.4f}) -> {'PASS' if turn_pass else 'FAIL'}",
        f"  [3] Differential Delta Steer   : {delta_steer:+.4f} rad",
        f"  [4] Turn Pre-Braking Throttle  : {turn_throt:+.4f} (Active Braking: {turn_throt < -0.30})",
        f"  [5] Corner Turn + Curvature    : {metrics['mu_corner_steer']:+.4f} rad (Throttle: {metrics['mu_corner_throt']:+.4f})",
        f"  [6] Exploration Noise (sigma)  : steer={sigma_steer:.4f}, throt={metrics['sigma_throt']:.4f}",
        f"  [7] Mirror Equivariance Error  : {metrics.get('mirror_equiv_err', 0.0):.6e}",
        "-" * 76,
        f"  OVERALL GATE RESULT: {'PASS (POLICY IS NEUTRAL & RESPONSIVE)' if passed else 'FAIL (BIAS EXCEEDS TOLERANCE)'}",
        "=" * 76,
    ]
    report = "\n".join(lines)

    if strict and not passed:
        failure_reasons = []
        if not neutral_pass:
            dev = abs(neutral_steer) - max_steer_bias
            failure_reasons.append(
                f"Straightaway bias {neutral_steer:+.4f} rad exceeds limit {max_steer_bias:.4f} rad by {dev:+.4f} rad"
            )
        if not turn_pass:
            deficit = min_turn_steer - turn_steer
            failure_reasons.append(
                f"Turn steering {turn_steer:+.4f} rad below authority requirement {min_turn_steer:.4f} rad by {deficit:+.4f} rad"
            )
        raise AssertionError(
            f"Straightaway Bias Gate FAILED for {ckpt_path}:\n  "
            + "\n  ".join(failure_reasons)
        )

    return passed, metrics, report


def test_straightaway_bias_gate(tmp_path):
    """Deterministic CPU regression test for checkpoint_probe + debias_checkpoint."""
    obs_space = gym.spaces.Box(low=-float("inf"), high=float("inf"), shape=(25100,))
    act_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(2,))
    actor = ARCProActor(obs_space, act_space, device="cpu", equivariant=False)

    with torch.no_grad():
        actor.policy_head[0].weight.zero_()
        actor.policy_head[0].bias.zero_()
        actor.policy_head[2].weight.zero_()
        actor.policy_head[2].bias.zero_()
        # Give unit 0 strong response to turn_token = -1.0 (col 64)
        actor.policy_head[0].weight[0, 64] = -1.0
        actor.policy_head[2].weight[0, 0] = 1.0
        actor.policy_head[2].weight[1, 0] = -0.8
        # Inject straightaway bias on b2[0] AND large weight on masked col 70 (prev_drive)
        actor.policy_head[2].bias[0] = 0.35
        actor.policy_head[0].weight[1, 70] = 50.0
        actor.policy_head[2].weight[0, 1] = 50.0

    biased_ckpt = str(tmp_path / "agent_biased.pt")
    torch.save({"policy": actor.state_dict()}, biased_ckpt)

    # 1. Verify biased checkpoint fails strict bias gate before debiasing
    passed_before, metrics_before, _ = run_bias_gate_check(
        biased_ckpt, max_steer_bias=0.05, min_turn_steer=0.30, strict=False
    )
    assert not passed_before
    assert abs(metrics_before["mu_neutral_steer"]) > 0.05

    # 2. Run debias_checkpoint and verify strict gate passes after debiasing
    debiased_ckpt = debias_checkpoint(biased_ckpt)
    passed_after, metrics_after, report = run_bias_gate_check(
        debiased_ckpt, max_steer_bias=0.05, min_turn_steer=0.30, strict=True
    )
    print("\n" + report)
    assert passed_after
    assert abs(metrics_after["mu_neutral_steer"]) <= 1e-5
    assert metrics_after["mu_turn_steer"] >= 0.30


def main():
    parser = argparse.ArgumentParser(
        description="Deterministic Straightaway Bias Quality Gate"
    )
    parser.add_argument(
        "--checkpoint",
        type=str,
        default=None,
        help="Path to checkpoint. Defaults to latest checkpoint.",
    )
    parser.add_argument(
        "--threshold-steer",
        type=float,
        default=0.05,
        help="Maximum allowed absolute straightaway steer bias in radians (default: 0.05)",
    )
    parser.add_argument(
        "--min-turn-steer",
        type=float,
        default=0.30,
        help="Minimum required steer authority under turn token = -1.0 (default: 0.30)",
    )
    parser.add_argument(
        "--no-strict",
        action="store_true",
        help="Do not exit with code 1 if gate fails (informational / probe mode)",
    )
    args = parser.parse_args()

    ckpt_path = args.checkpoint or find_latest_checkpoint()
    strict = not args.no_strict

    try:
        passed, metrics, report = run_bias_gate_check(
            ckpt_path=ckpt_path,
            max_steer_bias=args.threshold_steer,
            min_turn_steer=args.min_turn_steer,
            strict=strict,
        )
        print(report)
        sys.exit(0 if passed else 1)
    except AssertionError as e:
        print(e, file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
