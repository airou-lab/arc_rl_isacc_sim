#!/usr/bin/env python3
"""
Surgical Policy Head Debiasing Utility for ARCPro RL Checkpoints.
Calibrates policy_head.2.bias[0] using real synthesized road images through the
ARCProActor visual frontend (ResNet-18 + SpatialSoftmax2d) and strictly masked
telemetry (ACTOR_MASKED_TELEMETRY_INDICES = (5, 6, 8, 9, 11) set to 0.0).
Resets Adam momentum moments ONLY on policy_head.2.bias while preserving
log_std_parameter moments intact.
"""

from __future__ import annotations

import argparse
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
    ACTOR_MASKED_TELEMETRY_INDICES,
    ARCProActor,
)
from arcproLab.scripts.checkpoint_probe import (  # noqa: E402
    _get_cached_road_image,
    build_masked_telemetry,
)


def _reset_policy_head_bias_moments_only(
    ckpt: Dict[str, Any], policy_sd: Dict[str, torch.Tensor]
) -> int:
    """Zero Adam exp_avg[0] and exp_avg_sq[0] ONLY for policy_head.2.bias, NEVER log_std_parameter."""
    tensor_keys = [
        k for k, v in policy_sd.items() if isinstance(v, torch.Tensor)
    ]
    b2_param_idx = (
        tensor_keys.index("policy_head.2.bias")
        if "policy_head.2.bias" in tensor_keys
        else None
    )
    log_std_param_idx = (
        tensor_keys.index("log_std_parameter")
        if "log_std_parameter" in tensor_keys
        else 0
    )

    reset_count = 0
    for opt_key in ("policy_optimizer", "optimizer"):
        opt_dict = ckpt.get(opt_key, None)
        if not isinstance(opt_dict, dict):
            continue
        state_dict = opt_dict.get("state", None)
        if not isinstance(state_dict, dict) or not state_dict:
            continue

        # Identify exact key for policy_head.2.bias in optimizer state
        target_state_keys = []
        if b2_param_idx is not None:
            for candidate_k in (b2_param_idx, str(b2_param_idx)):
                if (
                    candidate_k in state_dict
                    and candidate_k not in (log_std_param_idx, str(log_std_param_idx))
                ):
                    p_state = state_dict[candidate_k]
                    if (
                        isinstance(p_state, dict)
                        and "exp_avg" in p_state
                        and p_state["exp_avg"].shape == torch.Size([2])
                    ):
                        target_state_keys.append(candidate_k)

        if not target_state_keys:
            # Fallback: collect all 2-element parameter state entries EXCEPT log_std_parameter (index 0)
            size2_keys = [
                k
                for k, p_state in state_dict.items()
                if isinstance(p_state, dict)
                and "exp_avg" in p_state
                and p_state["exp_avg"].shape == torch.Size([2])
                and k not in (log_std_param_idx, str(log_std_param_idx), 0, "0")
            ]
            if size2_keys:
                target_state_keys.append(size2_keys[-1])

        for tk in target_state_keys:
            p_state = state_dict[tk]
            if "exp_avg" in p_state and p_state["exp_avg"].numel() >= 1:
                p_state["exp_avg"][0] = 0.0
            if "exp_avg_sq" in p_state and p_state["exp_avg_sq"].numel() >= 1:
                p_state["exp_avg_sq"][0] = 0.0
            reset_count += 1

    return reset_count


@torch.no_grad()
def debias_checkpoint(
    ckpt_path: str,
    output_path: str | None = None,
    out_path: str | None = None,
    vision_mode: str = "rgb",
    scenario: str = "yellow_L_white_R",
) -> str:
    if not os.path.isfile(ckpt_path):
        raise FileNotFoundError(f"Checkpoint not found: {ckpt_path}")

    resolved_out = output_path or out_path
    if resolved_out is None:
        base, ext = os.path.splitext(ckpt_path)
        output_path = f"{base}_debiased{ext}"
    else:
        output_path = resolved_out

    print("=" * 80)
    print(f"SURGICAL POLICY HEAD DEBIASING: {os.path.basename(ckpt_path)}")
    print("=" * 80)
    print(f"  Input Checkpoint : {ckpt_path}")
    print(f"  Output Checkpoint: {output_path}")
    print(f"  Vision Mode      : {vision_mode} (Scenario: {scenario})")

    try:
        ckpt = torch.load(ckpt_path, map_location="cpu", weights_only=False)
    except TypeError:
        ckpt = torch.load(ckpt_path, map_location="cpu")

    is_wrapped = isinstance(ckpt, dict) and "policy" in ckpt
    policy_sd = ckpt["policy"] if is_wrapped else ckpt

    obs_space = gym.spaces.Box(low=-float("inf"), high=float("inf"), shape=(25100,))
    act_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(2,))
    actor = ARCProActor(
        observation_space=obs_space,
        action_space=act_space,
        device="cpu",
        equivariant=True,
        vision_mode=vision_mode,
    )
    actor.load_state_dict(policy_sd, strict=False)
    actor.eval()

    w0 = policy_sd["policy_head.0.weight"]  # Shape (64, 76)
    b0 = policy_sd["policy_head.0.bias"]    # Shape (64)
    w2 = policy_sd["policy_head.2.weight"]  # Shape (2, 64)
    b2 = policy_sd["policy_head.2.bias"]    # Shape (2)

    # 1. Evaluate neutral straightaway observation on REAL synthesized road image
    # with all masked slots [5, 6, 8, 9, 11] strictly set to 0.0 (col 70 = prev_drive = 0.0!)
    img_neutral = _get_cached_road_image(scenario, vision_mode)
    tel_neutral = build_masked_telemetry(
        turn_token=0.0, go_signal=1.0, speed=0.50, lookahead_kappa=0.0
    )
    for idx in ACTOR_MASKED_TELEMETRY_INDICES:
        assert tel_neutral[0, idx].item() == 0.0

    k_neutral = actor.G(img_neutral)
    tel_neutral_masked = actor._mask_actor_telemetry(tel_neutral)
    neutral_76 = torch.cat([k_neutral, tel_neutral_masked], dim=-1)

    h_neutral = torch.relu(torch.matmul(neutral_76, w0.T) + b0)
    pre_act_neutral = torch.matmul(h_neutral, w2.T) + b2
    steer_neutral_before = torch.tanh(pre_act_neutral[0, 0]).item()
    throt_neutral_before = torch.tanh(pre_act_neutral[0, 1]).item()
    z0_steer = pre_act_neutral[0, 0].item()

    print("\n[Before Calibration]:")
    print(f"  policy_head.2.bias[0] (Steer Bias Parameter) : {b2[0].item():+.6f} rad")
    print(f"  Net Pre-Activation z_steer                   : {z0_steer:+.6f}")
    print(
        f"  Neutral Straightaway Steer Output            : {steer_neutral_before:+.6f} rad ({steer_neutral_before * 180.0 / 3.14159:+.2f}°)"
    )
    print(f"  Neutral Straightaway Throttle Output         : {throt_neutral_before:+.6f}")

    # 2. Evaluate Corner Turn observation on synthesized left_turn road image (turn_token = -1.0, kappa = +0.25)
    img_corner = _get_cached_road_image("left_turn", vision_mode)
    tel_corner = build_masked_telemetry(
        turn_token=-1.0, go_signal=1.0, speed=0.50, lookahead_kappa=0.25
    )
    k_corner = actor.G(img_corner)
    corner_76 = torch.cat([k_corner, actor._mask_actor_telemetry(tel_corner)], dim=-1)
    h_corner = torch.relu(torch.matmul(corner_76, w0.T) + b0)
    pre_act_corner = torch.matmul(h_corner, w2.T) + b2
    steer_corner_before = torch.tanh(pre_act_corner[0, 0]).item()
    print(
        f"  Corner Turn Steer Output (Before)            : {steer_corner_before:+.6f} rad ({steer_corner_before * 180.0 / 3.14159:+.2f}°)"
    )

    # 3. Apply exact surgical calibration to policy_head.2.bias[0]
    b2_calibrated = b2.clone()
    b2_calibrated[0] = b2[0] - z0_steer
    policy_sd["policy_head.2.bias"] = b2_calibrated

    # 4. Verify post-calibration outputs
    pre_act_neutral_after = torch.matmul(h_neutral, w2.T) + b2_calibrated
    steer_neutral_after = torch.tanh(pre_act_neutral_after[0, 0]).item()
    throt_neutral_after = torch.tanh(pre_act_neutral_after[0, 1]).item()

    pre_act_corner_after = torch.matmul(h_corner, w2.T) + b2_calibrated
    steer_corner_after = torch.tanh(pre_act_corner_after[0, 0]).item()

    print("\n[After Calibration]:")
    print(
        f"  policy_head.2.bias[0] (Calibrated)           : {b2_calibrated[0].item():+.6f} rad (Shift: {-z0_steer:+.6f} rad)"
    )
    print(
        f"  Neutral Straightaway Steer Output            : {steer_neutral_after:+.6f} rad (EXACT ZERO)"
    )
    print(
        f"  Neutral Straightaway Throttle Output         : {throt_neutral_after:+.6f} (UNMODIFIED)"
    )
    print(
        f"  Corner Turn Steer Output (After)             : {steer_corner_after:+.6f} rad ({steer_corner_after * 180.0 / 3.14159:+.2f}°)"
    )

    # 5. Save updated checkpoint and reset ONLY policy_head.2.bias Adam moments
    if is_wrapped:
        ckpt["policy"] = policy_sd
        n_reset = _reset_policy_head_bias_moments_only(ckpt, policy_sd)
        if n_reset > 0:
            print(
                f"  Reset Adam optimizer momentum moments for policy_head.2.bias ({n_reset} optimizer dict(s); log_std_parameter preserved intact)"
            )
    else:
        ckpt = policy_sd

    torch.save(ckpt, output_path)
    print(f"\n[SUCCESS] Calibrated checkpoint saved to:\n  {output_path}\n" + "=" * 80)
    return output_path


def main() -> None:
    parser = argparse.ArgumentParser(description="Surgical Policy Head Debiasing Utility")
    parser.add_argument(
        "--checkpoint", type=str, required=True, help="Input checkpoint path (.pt)"
    )
    parser.add_argument(
        "--output", type=str, default=None, help="Output checkpoint path (.pt)"
    )
    parser.add_argument(
        "--vision-mode",
        type=str,
        default="rgb",
        choices=["rgb", "yuv"],
        help="Camera preprocessing mode ('rgb' default or 'yuv').",
    )
    args = parser.parse_args()

    debias_checkpoint(
        args.checkpoint, args.output, vision_mode=args.vision_mode
    )


if __name__ == "__main__":
    main()
