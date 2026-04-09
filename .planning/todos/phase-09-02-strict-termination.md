# Concerns & Todo: Phase 09-02 Calibration

## User Concerns (April 8, 2026)
- **Robot Scale vs Lane Width:** Visually (wireframe), the 8x robot sits neatly in one lane. However, mathematical resets are triggering when centered. This suggests a mismatch between perception/telemetry and visual truth.
- **Grace Period Masking:** Current grace periods (20-100 steps) are masking the root cause of resets. We need to fix the math so the robot can land stably without needing long grace periods.
- **Environment Layout:** Double yellow in the middle, white lines on the edges. The goal is to stay in one lane (Right Lane).

## Immediate Analysis (Gemini)
- **Indexing Bug Found:** The termination manager was reporting a `LatErr Norm` of `0.2214` while telemetry showed `0.0907`. This indicates the termination logic was reading the wrong index from the observation buffer (likely reading a velocity or steer value as lateral error).
- **8x Scale Jitter:** High-frequency physics jitter at 8x scale is triggering resets even if the robot is "visually" safe.

## Tasks
- [x] **Task 1: Direct Termination Math**
  - Bypass `observation_manager` in `terminations.py`.
  - Calculate `LatErr` directly using `TrackManager` to eliminate indexing bugs.
  - *Status: COMPLETE*

- [x] **Task 2: Calibrate Lane Limit**
  - Use 0.6m as the "center-to-line" limit for the 8x robot with zeroed spawn.
  - If robot center is at 0.0m, this allows 0.6m of wiggle room.
  - *Status: COMPLETE*

- [x] **Task 3: Final GUI Verification**
  - Run without grace period (or minimal 20-step buffer).
  - Confirm robot can land and drive without immediate reset.
  - *Status: COMPLETE*
