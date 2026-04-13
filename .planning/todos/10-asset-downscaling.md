# Phase 10: Asset Downscaling (Priority)

## Goal
Revert the environment and robot to original 1.0x metric size (F1Tenth scale) to eliminate complex normalization and simplify physics.

## Tasks
- [ ] **10-01-usd-isolation**: Modify track USD to remove all non-road/marker terrain.
- [ ] **10-02-global-downscale**: Scale USD and waypoint data to 1.0x.
- [ ] **10-03-robot-revert**: Revert robot scale to (1.0, 1.0, 1.0) and recalibrate mass (20kg chassis).
- [ ] **10-04-cfg-cleanup**: Remove `0.125` observation normalization from `arcpro_env_cfg.py`.
- [ ] **10-05-verify-downscale**: Confirm stable spawn and camera alignment in 1.0x world.

## Success Criteria
- [ ] Robot is ~0.45m wide (1.0x).
- [ ] Observation space matches physical reality without normalization coefficients.
- [ ] Stable 20kg physics with recalibrated actuator gains.
