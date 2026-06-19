# Project State: ARCPro RL v2.9 (Hardened Mastery / Gold Master)

**RESUME HERE**
- **Milestone**: Milestone 3: HD Perception & Production Hardening
- **Phase**: Phase 16: MARL Transition
- **Next Todo**: Wait for user to verify the dry run (`verify_policy.py --max_steps 50`) and confirm if pure vision lane following training started successfully. Once confirmed, proceed with MARL Architecture Discussion.

---

## Current Milestone
**Milestone 3: HD Perception & Production Hardening** (ACTIVE)

## Current Phase
**Phase 16: MARL Transition** (ACTIVE)

## Summary
The project has reached the **Gold Master** training baseline for Phase 16, building upon the Hardened Mastery v2.9 state. We have transitioned strictly to **Pure Vision-Based Lane Following**, completely removing reliance on waypoint track rewards to ensure the ResNet policy learns naturally without penalty-driven stagnation at curves.

Key Stabilizations (Gold Master Finalization):
- **Upright Physics & Settle**: Solved suspension-bounce reset by lowering spawn height to 12cm and height termination threshold to 2cm.
- **Ground Settle Logic**: Intercepted actions in `WaypointTrackingWrapper`, forcing 0.0 actions until robot Z height < 0.10m.
- **Vision FOV**: Camera re-mounted to `pos=(-0.3, 0.0, 0.35)` and rotated 180 deg to correctly face forward relative to the chassis (-X axis).
- **Pure Vision Lane Following**: Set weights for `lateral_error` and `heading` rewards to 0.0, to stop the agent from receiving penalties at curves due to straight-line waypoints.
- **Dynamic Speed Reward**: Updated `speed_reward` to purely reward the robot's local forward velocity (-X), completely detaching it from the old waypoint tangent logic.

## Recent Activity
- **Reward Fix**: Increased `stationary` penalty weight to 15.0 to resolve 'Lazy Bureaucrat' stagnation (+11/step minimum by standing still).
- **Watchdog Fix**: Patched regex bug in `watchdog.py` that misparsed scientific notation for episode length.
- **Monitoring**: Updated `LOG_FILE` pointer in watchdog to track `production_mastery_8_relaunch.log`.
- **System Cleanup**: Ran GSD health and consistency checks; codebase map marked for subagent refresh.

## Reference State (Gold Master)
- **Model**: ResNet-18 (Fusion Policy)
- **Scale**: 1.0x True Physics
- **Lateral Plateau**: 0.05m (Strict)
- **Smoothness Weight**: 15.0
- **Entropy Coef**: 0.01
- **Spawn Height**: 0.12m
- **Boundary Threshold**: 0.12m
- **Height Termination**: 0.02m
- **Envs**: 32
- **Sensors**: Enabled (Camera 224x224, tilted 30deg down)

## Active Todos (Queue)
1. [x] **16-01-REWARD-STABILITY**: Fix Greed Bug and Lazy Bureaucrat reward imbalances.
2. [x] **16-02-PROGRESS-REWARD**: Implement track-tangent projection for speed.
3. [x] **16-03-PHYSICS-STABILITY**: Confirm Gold Master orientation and drop logic.
4. [ ] **16-04-MARL-ARCHITECTURE**: Discuss refactoring RoadGraph to agent-indexed tensors.
