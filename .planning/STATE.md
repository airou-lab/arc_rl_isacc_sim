# Project State: ARCPro RL v2.7-highspeed

## Current Milestone
**Milestone 3: HD Perception & Production Hardening** (ACTIVE)

## Current Phase
**Phase 16: MARL Transition** (ACTIVE)

## Summary
The project has pivoted to **High-Speed Lapping** (Target: 3.0 m/s) to break the "Cowardly Local Minimum". We have successfully resolved critical physics inversions (Drive Scale: -40.0) and track alignment offsets (Regenerated 2.5m displacement cache). Training is now running with a massive **Stationary Penalty** (Standing still for 1s = -25,000 pts) and vision-only enforcement.

## Recent Activity
- **Physics Stabilization**: Fixed inverted drive polarity and restricted actions to [0, 1] range.
- **Alignment Fix**: Deleted stale 2.5m-offset track cache; regenerated high-fidelity boundary markers.
- **Forced Exploration**: Implemented extreme stationary penalties to break start-line stalling.
- **Vision Mandate**: Masked ground-truth telemetry (Lat/Head Error) to force ResNet-18 feature extraction.

## Reference State (v2.7)
- **Model**: ResNet-18 (Fusion Policy)
- **Scale**: 1.0x True Physics
- **Drive**: -40.0 Scale (Forward-only)
- **Rewards**: High-Speed Balance (Death: -25,000, Speed Weight: 100.0, Stationary: -100/step)

## Active Todos (Queue)
1. [x] **16-01-PHYSICS**: Fix drive polarity and clamping for high-speed stability.
2. [x] **16-02-CACHE**: Regenerate track boundary cache to fix 2.5m displacement.
3. [ ] **16-03-SKRL**: Integrate SKRL backend for multi-agent policy training. (NEXT)
4. [ ] **17-competitive-racing**: Overtaking and multi-agent interaction logic.

**RESUME HERE**
- Milestone: Milestone 3
- Phase: Phase 16 (MARL Transition)
- Next Todo: Monitor `v3_stable_forcing` training until `ep_len_mean > 550` (First Lap).
