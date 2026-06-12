# Project State: ARCPro RL v2.8-marl-stabilized

**RESUME HERE**
- **Milestone**: Milestone 3
- **Phase**: Phase 16 (MARL Transition)
- **Next Todo**: Proceed with MARL Architecture Discussion and training. Single-agent physics is stable using the Control Flip and Rolling Start strategy, ready to expand to multi-agent HD vision with Adaptive CNN. Focus on refactoring RoadGraph from singleton to agent-indexed tensors and resolving VRAM constraints.

---

## Current Milestone
**Milestone 3: HD Perception & Production Hardening** (ACTIVE)

## Current Phase
**Phase 16: MARL Transition** (ACTIVE)

## Summary
Phase 16 single-agent physics is now stabilized and ready for training. We discovered the `F1Tenth_Metric.usd` was built upside-down and fixed the resultant physics crashes by accepting the native orientation and using a "Control Flip" strategy. The camera and visuals have been aligned to the flipped chassis, and stagnation limits have been successfully tuned with a rolling start to allow the agent to learn.

Physics stabilization achieved:
- **Upside-Down USD**: Accepted native upside-down spawn orientation in `events.py` to preserve PhysX suspension and avoid NaN crashes.
- **Control Flip**: Flipped the drive control polarity (`drive.scale=-20.0`) in `arcpro_env_cfg.py`. Positive throttle now successfully pushes the upside-down chassis forward.
- **Camera & Visuals**: Camera mounted at X=-0.3, rotated 180 degrees to look forward. Added a highly visible red CuboidCfg marker to trace direction.
- **Stagnation Limits**: Strict 10-step stagnation limit is active, supplemented by a 0.2-0.5 m/s rolling start to prevent immediate termination of the 5kg chassis.
- **Config Decorators**: Verified all configuration classes have the `@configclass` decorator to prevent KeyError 'visual'.

## Recent Activity (Phase 16 Stabilization)
- **Training Convergence**: Reached ~300k steps with ep_len_mean ~40 steps, std decayed to 0.66, and solidly positive explained_variance (0.33).
- **Physics Fix (Control Flip)**: Implemented "Control Flip" by reverting to native USD orientation and setting `drive.scale` to `-20.0`.
- **Termination Update**: Tuned stagnation limits to strict 10-steps with absolute velocity tracking, aided by a rolling start.
- **Camera Configuration**: Mounted camera correctly on the upside-down chassis and added red visual trace marker.

## Reference State (v2.8)
- **Model**: ResNet-18 (Fusion Policy)
- **Scale**: 1.0x True Physics
- **Orientation**: Native Upside-Down (WXYZ math: `W=cos(half_yaw), X=0.0, Y=0.0, Z=sin(half_yaw)`)
- **Drive Polarity**: `scale=-20.0` (Flipped to compensate for upside-down chassis)
- **Spawn Height**: 0.12m
- **Damping**: 1.0 (Throttle) / 2.0 (Steering)
- **Envs**: 8-16
- **Sensors**: Enabled (Camera 224x224)

## Active Todos (Queue)
1. [x] **16-01-PHYSICS**: Fix drive polarity and clamping for high-speed stability.
2. [x] **16-02-CACHE**: Regenerate track boundary cache to fix 2.5m displacement.
3. [x] **16-03-MARL-STABILITY**: Fix orientation, scaling, and gate visuals.
4. [x] **16-04-FRESH-TRAIN**: Monitor Step 0 training curve and implement Control Flip for stability.
