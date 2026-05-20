# Project Context: ARCPro RL v2.7

## Environment State
- **Scale**: 1.0x Metric (True Physics).
- **Mode**: "Full Fidelity" (Lines and Boundaries active).
- **Orientation**: **South-facing** (-1.57 rad) at spawn.
- **Target**: Lane Centering with High-Speed (3.0 m/s) requirement.

## Physical Constants (v2.7)
- **Mass**: 4.092kg (F1Tenth specification).
- **Wheelbase**: 0.33m.
- **Drive Scale**: **-40.0** (Negative maps positive action to forward motion).
- **Throttle Range**: [0, 1] (Forward only, strictly clamped).
- **Control Rate**: 50Hz (Env) / 20Hz (Policy).

## Perception & Policy
- **Backbone**: ResNet-18 (Pre-trained ImageNet, Fine-tuning active).
- **Vision**: 640x360 -> Resized to 224x224 (Fusion Policy).
- **Masking**: Ground-truth `lat_err` and `head_err` are **MASKED** in observations.
- **Telemetry**: 12-element vector fused with vision features.

## Reward Mechanics
- **Death Penalty**: -25,000 (Triggered by white line contact).
- **Stationary Penalty**: -100 per step (if speed < 0.5 m/s).
- **Speed Reward**: Linear (+100 weight) for forward velocity.

## Track Info
- **Length**: 32.16 meters.
- **Lap Threshold**: `ep_len_mean > 536 steps` (at 3.0 m/s).
- **Cache**: `arcproLab/mdp/track_boundaries_1x.npz` (Regenerated 2026-05-19).
