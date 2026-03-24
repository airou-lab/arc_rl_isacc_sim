---
status: testing
phase: 04-robot-refinement-verification
source: [04-01-PLAN.md, 04-02-PLAN.md]
---

## Current Test

number: 1
name: Robot Metric Validation
expected: |
  Running `./isaaclab.sh -p arcproLab/scripts/verify_robot.py` (or check_map_scale) reports the correct physical sizes: a wheelbase of 25cm, track width of 24cm, and the robot's overall width relative to the road is appropriate.
awaiting: user response

## Tests

### 1. Robot Metric Validation
expected: Running the verification scripts shows the robot is scaled to the correct physical metrics (25cm wheelbase, 403mm x 287mm bounding box, 4.092kg mass).
result: pending

### 2. Free Camera Viewer
expected: When running the environment with rendering, the Isaac Lab viewer tracks the robot but allows manual free-flight navigation for debugging.
result: pending

### 3. Physics Clipping & Stability
expected: Running `./isaaclab.sh -p arcproLab/scripts/check_clipping.py` completes the 100-step drop test without any NaNs (physics explosions) and settles at a stable height without sinking into the floor.
result: pending

## Summary

total: 3
passed: 0
issues: 0
pending: 3
skipped: 0

## Gaps

