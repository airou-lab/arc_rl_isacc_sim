#!/usr/bin/env python3
"""Real-image CPU policy probe for ARCPro reinforcement learning checkpoints.

Evaluates the full ARCProActor pipeline (ResNet-18 + SpatialSoftmax2d keypoint
frontend + masked 12D telemetry + smooth get_log_std() bijection) on deterministic
synthesized road scenes ("centered_sym", "yellow_L_white_R", "white_L_yellow_R",
"left_turn", "right_turn").
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from typing import Any, Dict

import gymnasium as gym
import torch

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ARCPRO_LAB_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
WORKSPACE_ROOT = os.path.abspath(os.path.join(ARCPRO_LAB_DIR, ".."))
for _p in (WORKSPACE_ROOT, ARCPRO_LAB_DIR, SCRIPT_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from arcproLab.agents.skrl_models import (  # noqa: E402
    ACTION_MIRROR_SIGNS,
    ACTOR_MASKED_TELEMETRY_INDICES,
    TELEMETRY_DIM,
    TELEMETRY_MIRROR_SIGNS,
    ARCProActor,
    synthesize_road_image,
)

# Shared module-level cache for deterministic synthesized road images on CPU
_ROAD_IMAGE_CACHE: Dict[tuple[str, str], torch.Tensor] = {}


def _get_cached_road_image(scenario: str, vision_mode: str) -> torch.Tensor:
    key = (scenario.lower(), vision_mode.lower())
    if key not in _ROAD_IMAGE_CACHE:
        _ROAD_IMAGE_CACHE[key] = synthesize_road_image(
            batch_size=1,
            scenario=scenario,
            vision_mode=vision_mode,
            device="cpu",
        )
    return _ROAD_IMAGE_CACHE[key]


def build_masked_telemetry(
    turn_token: float = 0.0,
    go_signal: float = 1.0,
    speed: float = 0.50,
    lookahead_kappa: float = 0.0,
    device: str | torch.device = "cpu",
) -> torch.Tensor:
    """Construct a (1, 12) telemetry vector with all ACTOR_MASKED_TELEMETRY_INDICES strictly 0.0."""
    tel = torch.zeros((1, TELEMETRY_DIM), dtype=torch.float32, device=device)
    tel[0, 0] = float(turn_token)
    tel[0, 1] = float(go_signal)
    tel[0, 3] = float(speed)
    tel[0, 10] = float(lookahead_kappa)
    for idx in ACTOR_MASKED_TELEMETRY_INDICES:
        tel[0, idx] = 0.0
    return tel


@torch.no_grad()
def evaluate_checkpoint(
    ckpt_path: str,
    vision_mode: str = "rgb",
    equivariant: bool = False,
    neutral_scenario: str = "yellow_L_white_R",
) -> Dict[str, Any]:
    """Load policy weights onto CPU and evaluate real-image steering bias, turn authority, and mirror symmetry."""
    if not os.path.isfile(ckpt_path):
        raise FileNotFoundError(f"Checkpoint not found: {ckpt_path}")

    try:
        ckpt = torch.load(ckpt_path, map_location="cpu", weights_only=False)
    except TypeError:
        ckpt = torch.load(ckpt_path, map_location="cpu")

    if isinstance(ckpt, dict):
        if "policy" in ckpt:
            sd = ckpt["policy"]
        elif "models" in ckpt and isinstance(ckpt["models"], dict) and "policy" in ckpt["models"]:
            sd = ckpt["models"]["policy"]
        else:
            sd = ckpt
    else:
        sd = ckpt

    obs_space = gym.spaces.Box(low=-float("inf"), high=float("inf"), shape=(25100,))
    act_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(2,))

    actor = ARCProActor(
        observation_space=obs_space,
        action_space=act_space,
        device="cpu",
        equivariant=True,
        vision_mode=vision_mode,
    )
    actor.load_state_dict(sd, strict=False)
    actor.eval()

    # Compute runtime exploration standard deviation via smooth tanh bijection get_log_std()
    log_std = actor.get_log_std()
    std = torch.exp(log_std).cpu().numpy()

    def _eval_actor_scene(scenario: str, tel: torch.Tensor, use_equiv_head: bool) -> torch.Tensor:
        img = _get_cached_road_image(scenario, vision_mode)
        tel_masked = actor._mask_actor_telemetry(tel)
        if use_equiv_head:
            actor.equivariant = True
            act = actor.forward_from_image(img, tel_masked)
        else:
            # Extract keypoints from real synthesized road image via G(I), then evaluate raw policy_head
            actor.equivariant = True
            k = actor.G(img)
            raw_logits = actor.policy_head(torch.cat([k, tel_masked], dim=-1))
            act = torch.tanh(raw_logits)
        return act.squeeze(0)

    tel_neutral = build_masked_telemetry(
        turn_token=0.0, go_signal=1.0, speed=0.50, lookahead_kappa=0.0
    )
    tel_turn = build_masked_telemetry(
        turn_token=-1.0, go_signal=1.0, speed=0.50, lookahead_kappa=0.0
    )
    tel_corner = build_masked_telemetry(
        turn_token=-1.0, go_signal=1.0, speed=0.50, lookahead_kappa=0.25
    )
    tel_right_corner = build_masked_telemetry(
        turn_token=1.0, go_signal=1.0, speed=0.50, lookahead_kappa=-0.25
    )

    act_neutral = _eval_actor_scene(neutral_scenario, tel_neutral, equivariant)
    act_sym = _eval_actor_scene("centered_sym", tel_neutral, equivariant)
    act_yw = _eval_actor_scene("yellow_L_white_R", tel_neutral, equivariant)
    act_wy = _eval_actor_scene("white_L_yellow_R", tel_neutral, equivariant)
    act_turn = _eval_actor_scene("left_turn", tel_turn, equivariant)
    act_corner = _eval_actor_scene("left_turn", tel_corner, equivariant)
    act_right_corner = _eval_actor_scene("right_turn", tel_right_corner, equivariant)

    # Measure exact equivariant mirror error across all 5 road scenarios
    actor.equivariant = True
    tel_signs = torch.tensor(TELEMETRY_MIRROR_SIGNS, dtype=torch.float32)
    act_signs = torch.tensor(ACTION_MIRROR_SIGNS, dtype=torch.float32)
    scenario_pairs = [
        ("centered_sym", tel_neutral),
        ("yellow_L_white_R", tel_neutral),
        ("white_L_yellow_R", tel_neutral),
        ("left_turn", tel_corner),
        ("right_turn", tel_right_corner),
    ]
    max_equiv_err = 0.0
    for scen_name, t_vec in scenario_pairs:
        img = _get_cached_road_image(scen_name, vision_mode)
        img_flip = torch.flip(img, dims=[-1])
        t_mir = t_vec * tel_signs
        mu_orig = actor.forward_from_image(img, t_vec)
        mu_flip = actor.forward_from_image(img_flip, t_mir)
        err = float((mu_flip - act_signs * mu_orig).abs().max().item())
        max_equiv_err = max(max_equiv_err, err)

    bias = float(act_neutral[0].item())
    turn_steer = float(act_turn[0].item())
    delta_steer = turn_steer - bias
    pre_brake = float(act_turn[1].item())

    return {
        "checkpoint": os.path.basename(ckpt_path),
        "path": os.path.abspath(ckpt_path),
        "vision_mode": vision_mode,
        "equivariant": bool(equivariant),
        "mu_neutral_steer": round(bias, 4),
        "mu_neutral_throt": round(float(act_neutral[1].item()), 4),
        "mu_centered_sym_steer": round(float(act_sym[0].item()), 4),
        "mu_centered_sym_throt": round(float(act_sym[1].item()), 4),
        "mu_yellow_l_white_r_steer": round(float(act_yw[0].item()), 4),
        "mu_yellow_l_white_r_throt": round(float(act_yw[1].item()), 4),
        "mu_white_l_yellow_r_steer": round(float(act_wy[0].item()), 4),
        "mu_white_l_yellow_r_throt": round(float(act_wy[1].item()), 4),
        "mu_turn_steer": round(turn_steer, 4),
        "mu_turn_throt": round(pre_brake, 4),
        "delta_turn_steer": round(delta_steer, 4),
        "mu_corner_steer": round(float(act_corner[0].item()), 4),
        "mu_corner_throt": round(float(act_corner[1].item()), 4),
        "mu_right_turn_steer": round(float(act_right_corner[0].item()), 4),
        "mu_right_turn_throt": round(float(act_right_corner[1].item()), 4),
        "sigma_steer": round(float(std[0]), 4),
        "sigma_throt": round(float(std[1]), 4),
        "mirror_equiv_err": round(max_equiv_err, 6),
        "mirror_equivariance_error": round(max_equiv_err, 6),
        "is_neutral": abs(bias) <= 0.02,
        "is_braking_in_turn": pre_brake < -0.30,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="ARCPro Real-Image Checkpoint Weight Probe")
    parser.add_argument("checkpoint", type=str, help="Path to agent_*.pt")
    parser.add_argument("--json", action="store_true", help="Output raw JSON")
    parser.add_argument(
        "--vision-mode",
        type=str,
        default="rgb",
        choices=["rgb", "yuv"],
        help="Camera preprocessing mode ('rgb' default or 'yuv').",
    )
    parser.add_argument(
        "--equivariant",
        dest="equivariant",
        action="store_true",
        default=False,
        help="Evaluate with full equivariant policy head symmetrization enabled.",
    )
    parser.add_argument(
        "--no-equivariant",
        dest="equivariant",
        action="store_false",
        help="Evaluate raw policy_head output on equivariant visual keypoints (default).",
    )
    args = parser.parse_args()

    try:
        metrics = evaluate_checkpoint(
            args.checkpoint,
            vision_mode=args.vision_mode,
            equivariant=args.equivariant,
        )
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
    print(
        f"ARCPro Policy Probe: {metrics['checkpoint']} "
        f"(vision={metrics['vision_mode']}, equivariant={metrics['equivariant']})"
    )
    print("=" * 70)
    print(f"  Neutral Straight Steer: {b:+.4f} (Target |bias| <= 0.02) -> {target_str}")
    print(f"  Neutral Straight Throt: {metrics['mu_neutral_throt']:+.4f}")
    print(
        f"  Yellow-L / White-R:     Steer={metrics['mu_yellow_l_white_r_steer']:+.4f}, "
        f"White-L / Yellow-R: Steer={metrics['mu_white_l_yellow_r_steer']:+.4f}"
    )
    print(
        f"  Turn Token -1.0 Steer:  {metrics['mu_turn_steer']:+.4f} (ΔSteer = {metrics['delta_turn_steer']:+.4f})"
    )
    print(f"  Turn Token -1.0 Throt:  {metrics['mu_turn_throt']:+.4f} -> {brake_str}")
    print(
        f"  Corner Turn+Curve:      Steer={metrics['mu_corner_steer']:+.4f}, Throt={metrics['mu_corner_throt']:+.4f}"
    )
    print(
        f"  Mirror Equiv Error:     {metrics['mirror_equiv_err']:.6f}"
    )
    print(
        f"  Exploration Noise:      Steer σ={metrics['sigma_steer']:.4f}, Throt σ={metrics['sigma_throt']:.4f}"
    )
    print("=" * 70)


if __name__ == "__main__":
    main()
