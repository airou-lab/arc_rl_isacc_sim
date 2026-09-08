# RESUME — Context for Next Agent Session
**Last Updated:** 2026-09-08
**Active Branch:** `feature/realsense-realistic-kinematics`

---

## 1. Executive Summary & Breakthrough Milestones

### Peak Telemetry Achieved (>4.14M Steps / 82.8M Transitions)
- **Episode Reward (`Rew`):** **`+4,767.8`** (All-time record peak).
- **Episode Length (`Len`):** Broken the 1,000-step barrier! Peaking at **`1,066 steps`** (`agent_4141500_peak.pt`).
- **Speed (`Spd`):** Cruising smoothly at **`0.70 – 0.86 m/s`**.
- **Active Branch**: `feature/realsense-realistic-kinematics` (isolated from `main`).
- **Checkpoints**: Saved to branch-isolated directory `logs/ppo_skrl_feature_realsense-realistic-kinematics/`.

---

## 2. Root Cause Fixes Applied in This Project

### Issue 104: Physical Kinematics Unlocking, RealSense D455 Bumper Mounting & Racing Line Slack
- **Problem:** Across 5M steps, camera-only lane following failed to clear Corner 1 ($R = 0.464\text{ m}$). A 5-worker divergent CTA Swarm audit revealed:
  1. Wheelbase $L = 0.33\text{ m}$ with `max_steering_angle = 0.50\text{ rad}` yields $R_{\min} = 0.604\text{ m}$. Centerline following forced an unavoidable $14\text{ cm}$ outward drift into the outer line ($6.3\text{ cm}$ clearance).
  2. `arcproLab/mdp/spawner.py` hardcoded USD RevoluteJoint limits to $\pm 35.0^\circ$. For Ackermann steering $\delta = 0.68\text{ rad}$ ($39^\circ$), the inner wheel requires $50.9^\circ$. PhysX was clamping inner wheel rotation to $35^\circ$, making tight turns physically impossible.
  3. Real D435i RGB is only $69.4^\circ$ with rolling shutter (motion blur). Intel RealSense D455 has true $86\text{--}90^\circ$ FOV with Global Shutter. Camera was floating $25\text{ cm}$ in mid-air in front of the vehicle at $0.35\text{ m}$ height (`pos=(0.5, 0, 0.35)`).
  4. Real racers use an "out-in-out" racing line through sharp $90^\circ$ corners ($R_{\text{eff}} \ge 0.70\text{ m}$). Linear `lateral_error_reward` strictly penalized any deviation from the centerline.
- **Resolution (Branch `feature/realsense-realistic-kinematics`):**
  1. Raised `spawner.py` RevoluteJoint limits from $\pm 35.0^\circ$ to $\pm 55.0^\circ$.
  2. Raised `max_steering_angle` from $0.50\text{ rad}$ to $0.68\text{ rad}$ ($39.0^\circ$), unlocking $R_{\min} = 0.408\text{ m} < 0.464\text{ m}$.
  3. Repositioned camera to realistic front bumper mount `pos=(0.22, 0.0, 0.18)` with $15^\circ$ downward pitch. Updated visual marker to match.
  4. In `lateral_error_reward`, added dynamic curvature slack $\text{slack} = \text{clamp}(\kappa \times 0.08, 0, 0.10)$. Straights enforce strict $0\text{ cm}$ centering; sharp corners allow up to $10\text{ cm}$ of apex-cutting without penalty.
  5. Configured automated branch-prefixed experiment directory (`logs/ppo_skrl_<branch>/`) so branch runs never collide with `main`.
  6. Warm-started transfer from landmark peak checkpoint `agent_4141500_peak.pt` on `feature/realsense-realistic-kinematics`.

### Issue 103: Checkpoint Disk Bloat & 250k Pruning
- Pruned 1,230 redundant intermediate checkpoints (reclaimed 7.36 GB). Increased checkpoint interval to 250k steps. Archived landmark checkpoints to `archive/landmark_checkpoints/`.

### Issue 102: Leg 2 Centerline Defect & Recalibration Deployment
- Re-centered 1,142 defective waypoints along Leg 2 to true geometric midpoint between yellow and white road lines. Clearance restored from 0.1cm to 20.2cm.

---

## 3. Active Background Infrastructure

- **Active Git Branch:** `feature/realsense-realistic-kinematics`
- **Active Training Session:** Background tmux session `training` (20 camera envs, RTX 3060, warm-started from `agent_4141500_peak.pt`).
- **Branch Checkpoint Directory:** `logs/ppo_skrl_feature_realsense-realistic-kinematics/`
- **Autonomous Sentinel:** Running in tmux pane `training:0.1` (`interval=15.0s`).
- **5-Sector Certified Spawner Active:**
  1. Sector 1 (20%): Start Line `(-16.197, 5.50, yaw = -1.5708)` (11m straight)
  2. Sector 2 (20%): Corner 1 Approach `(-16.197, -3.50, yaw = -1.5708)` (2.5m runway)
  3. Sector 3 (20%): Leg 2 Post-C1 Exit `(-12.000, -7.10, yaw = 0.0)` (8m runway)
  4. Sector 4 (20%): Leg 2 Intersection Approach `(-8.000, -7.10, yaw = 0.0)` (4.15m runway)
  5. Sector 5 (20%): North Straight Return `(-6.082, 6.377, yaw = 3.14159)` (10m runway)
- **Landmark Baselines:** Archived in [`archive/landmark_checkpoints/`](file:///home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/archive/landmark_checkpoints/):
  - `agent_4141500_peak.pt` (All-time peak: 1,066 steps, +4,767.8 rew)
  - `agent_3670000_1000step_peak.pt` (1,001 steps peak)
  - `agent_4286500_latest.pt` (Latest main state)

---

## 4. Next Steps for Incoming Agent

1. **Monitor Warm-Start Benchmark Run:** Check `logs/skrl_phase1.log` to verify vehicle behavior through Corner 1 with expanded steering and bumper camera.
2. **Verify Corner 1 Transit:** Ensure `ep_len_mean` advances past 860 towards full-lap completion (>1,500 steps).
3. **Merge Back into Main:** Once verified, copy champion checkpoint to `archive/landmark_checkpoints/agent_realsense_kinematics_champion.pt` and merge `feature/realsense-realistic-kinematics` into `main`.
4. **Strict Rules:** Always adhere to GSD Auto-Resume Rule, Explicit Approval Rule, and Workspace Context Rule.




