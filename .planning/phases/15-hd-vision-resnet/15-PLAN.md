# Phase 15 Plan: HD Vision & Adaptive CNN

This phase upgrades the ARCPro RL vision pipeline to **640x360** using a robust **Adaptive CNN** backbone with **Vision-Dominant Fusion** to solve the "fear-based" driving and brittle survival issues while staying within VRAM limits.

## Goals
- [x] **HD Architecture**: Implement Adaptive CNN with Early Pooling in `fusion_policy.py` to handle 640x360 pixels.
- [x] **Vision-First Fusion**: Implement a high-ratio concatenation to ensure pixels are the primary navigation signal.
- [ ] **Hardened Spawn**: Finalize the Domain Randomization in `events.py` (Offset, Heading, Velocity).
- [x] **VRAM Optimization**: Configure `arcpro_env_cfg.py` for 8 environments at 640x360 resolution.
- [ ] **Momentum Convergence**: Achieve 120s full-track survival at speeds > 1.5 m/s.

## Implementation Tasks

### 1. The Adaptive Brain (`fusion_policy.py`)
- Implement a `FlexibleFeaturesExtractor` using `AdaptiveAvgPool2d((128, 128))` as the first layer.
- This ensures the model is stable regardless of input resolution (640x360 or 960x540).
- Preserve 256-dim latent fusion head.

### 2. Vision-Dominant Fusion
- Inputs: 256-dim CNN features + 12-dim Telemetry.
- Logic: Concatenate features and map to a 256-dim latent space.
- Scaling: Apply LayerNorm to stabilize multi-modal signals before the policy head.

### 3. Environment & Training Config
- Update `arcpro_env_cfg.py`:
    - `width=640, height=360`.
    - `num_envs=8` (current stable config).
    - `decimation=10` (50Hz control).
- Update `train_policy.py` hyperparameters:
    - `n_steps=512` (System RAM safety).
    - `batch_size=32` (VRAM backprop safety).
    - `learning_rate=2e-4`.

## Verification Metrics (UAT)
- **Memory Check**: VRAM usage < 11.0 GB on RTX 3060.
- **Pace Check**: Mean speed > 1.0 m/s within 500k steps.
- **Robustness Check**: Survive randomized spawns (±8cm offset) within 1M steps.

## Resume Here
1. Monitoring **v7 Production Run** (640x360, 8 envs, Adaptive CNN).
2. Survival at ~3 seconds (143 steps) achieved at 63k steps.
3. Entropy is dropping, indicating convergence on a driving strategy.
