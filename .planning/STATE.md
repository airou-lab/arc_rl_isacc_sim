# Project State: ARCPro RL v2.8-marl-stabilized

**RESUME HERE**
- **Milestone**: Milestone 3
- **Phase**: Phase 16 (MARL Transition)
- **Next Todo**: Monitor fresh training run from Step 0. The uninitialized neural network is currently in the "Drunk Driving" phase applying maximum steering, expected to improve over the next 500k steps.

---

## Current Milestone
**Milestone 3: HD Perception & Production Hardening** (ACTIVE)

## Current Phase
**Phase 16: MARL Transition** (ACTIVE)

## Summary
Phase 16 training has been restarted from Step 0. The previous checkpoint was identified as "poisoned" (trained while the USD asset was upside-down), necessitating a purge of all old logs/ppo checkpoints. A fresh run is currently active in the background.

Physics have been extensively stabilized for the new run:
- Robot spawn is now perfectly flat (12cm height, zero pitch, WXYZ North-facing quaternion applied via math).
- Rolling start (0.2 - 0.5 m/s) is enabled to beat the strict 10-step stagnation limit.
- roadmark_contact threshold relaxed from 0.15m to 0.05m to give the "drunk" Step 0 RL agent a physical buffer.
- drive.scale set to positive 20.0.

The fresh run is currently in the "Drunk Driving" phase (ep_len_mean ~8 steps, std ~1.0). This is expected behavior as the agent explores the narrow walls before optimization.

## Recent Activity (Phase 16 Stabilization)
- **Training Restart**: Purged old checkpoints, launched fresh Step 0 training.
- **Verified Scaling**: Synchronized track and robot scaling to 1.0x native metric consistency.
- **Fixed Orientation**: Corrected spawn heading to North-facing (1.5708 rad) for alignment with road graph. Applied exact WXYZ quaternion math.
- **Spawn Stability**: Robot spawns flat at 12cm height.
- **Physics Tuning**: drive.scale set to 20.0, rolling start (0.2-0.5 m/s) enabled.
- **Termination Threshold**: Active roadmark_contact termination with a relaxed 0.05m threshold to allow buffer.
- **Performance Optimization**: Tuned for 8-16 environments to ensure VRAM/FPS safety on RTX 3060 with HD cameras (224x224).

## Reference State (v2.8)
- **Model**: ResNet-18 (Fusion Policy)
- **Scale**: 1.0x True Physics
- **Orientation**: 1.5708 rad (North)
- **Spawn Height**: 0.12m
- **Damping**: 1.0 (Throttle) / 2.0 (Steering)
- **Envs**: 8-16
- **Sensors**: Enabled (Camera 224x224)

## Active Todos (Queue)
1. [x] **16-01-PHYSICS**: Fix drive polarity and clamping for high-speed stability.
2. [x] **16-02-CACHE**: Regenerate track boundary cache to fix 2.5m displacement.
3. [x] **16-03-MARL-STABILITY**: Fix orientation, scaling, and gate visuals.
4. [ ] **16-04-FRESH-TRAIN**: Monitor Step 0 training curve (Drunk Driving phase) and ensure ep_len_mean climbs above 10. (ACTIVE)
