# Project State: ARCPro RL v2.8-marl-stabilized

**RESUME HERE**
- **Milestone**: Milestone 3
- **Phase**: Phase 16 (MARL Transition)
- **Next Todo**: Continue monitoring the healthy training run which is successfully learning to drive. Consider introducing multi-agent elements now that single-agent physics is stable.

---

## Current Milestone
**Milestone 3: HD Perception & Production Hardening** (ACTIVE)

## Current Phase
**Phase 16: MARL Transition** (ACTIVE)

## Summary
Phase 16 training has been restarted from Step 0 and is now highly stabilized using the "Control Flip" strategy. The network is successfully learning to drive and avoid walls, demonstrating healthy convergence metrics (ep_len_mean ~40, explained_variance ~0.33 at 300k steps).

Physics stabilization achieved via "Control Flip":
- Reverted to native USD upside-down spawn orientation (`events.py`) because forcing it upright broke the PhysX suspension and caused NaN crashes.
- Flipped the drive control polarity: `drive.scale` is set to `-20.0` in `arcpro_env_cfg.py`. Positive throttle now spins wheels "backwards" pushing the upside-down chassis forward.
- Updated `stagnation_termination` to use absolute velocity (`torch.norm(vel[:, :2], dim=1)`) and relaxed the threshold to 50 steps (~1.0s) so it tracks movement correctly regardless of flipped local axes and gives the agent time to explore.
- roadmark_contact threshold remains at 0.05m to give the agent a physical buffer.

## Recent Activity (Phase 16 Stabilization)
- **Training Convergence**: Reached ~300k steps with ep_len_mean ~40 steps, std decayed to 0.66, and solidly positive explained_variance (0.33).
- **Physics Fix (Control Flip)**: Implemented "Control Flip" by reverting to native USD orientation and setting `drive.scale` to `-20.0`.
- **Termination Update**: Increased stagnation limit to 50 steps and converted to absolute velocity check.

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
