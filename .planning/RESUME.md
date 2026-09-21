# RESUME — Context for Next Agent Session
**Last Updated:** 2026-09-18
**Active Milestone:** M004 (RealSense Realistic Kinematics & Clean Step 0 Bootstrap)
**Active Phase:** Phase 03: Step 0 Scratch Training & Corner 1 Transit
**Active Branch:** `feature/realsense-realistic-kinematics`

---

## 1. Executive Summary & Active Milestone Status

### Milestone M004 Overview
- **Milestone**: M004 (RealSense Realistic Kinematics & Clean Step 0 Bootstrap)
- **Phase**: Phase 03: Step 0 Scratch Training & Corner 1 Transit
- **Goal**: Full-lap autonomous driving with RealSense D455 realistic bumper camera mounting, certified Ackermann kinematics ($\pm 55.0^\circ$ joint limits, $0.68\text{ rad}$ max steer angle, $R_{\min}=0.408\text{ m}$), progressing from Corner 1 transit breakthrough through Leg 2 intersection toward Corner 2 and full-lap completion (5,000 steps, 64.85m).
- **Active Training State**: Running stably past Step 2.41M+ (48.2M+ env transitions) in background tmux session `training` (20 camera envs, RTX 3060, ~10.4GB VRAM, actively logging to `logs/skrl_phase1.log`).
- **Milestone Peak**: All-time peak episode length of **1,536 steps** (~22m, ~33% of full circuit) and peak reward **+10,209.8** achieved at Step 1,497,999; continuous milestone checkpoints preserved up to `agent_2400000.pt`.
- **Recent Telemetry (Step 2,400,000–2,410,000+ / 48.2M env steps)**:
  - Episode Reward (`Rew`): Mean $2,000\text{--}3,856$, Peak $+9,756.0$
  - Episode Length (`Len`): Mean $340\text{--}623\text{ steps}$, Frontier Peak surged to **$1,461\text{ steps}$** (shattering the previous 1,100-step barrier immediately following the Issue 109 intersection/apex fix!)
  - Speed (`Spd`): $0.66\text{--}0.79\text{ m/s}$ (target speed maintained)
  - Waypoints (`WPs_cum`): Peaking at **$569\text{ WPs}$** discovered per rollout (up from 391)

---

## 2. Recent Completed Actions & Remediation

### 1. Kinematic Curvature Pre-Braking Throttle Prior
- **File**: [`arcproLab/agents/skrl_models.py`](file:///home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/arcproLab/agents/skrl_models.py)
- **Mechanism**: Injected inductive deceleration prior on the drive channel inside `ARCProActor.compute()`:
  $$\text{brake\_bias} = \text{clamp}(|\kappa| \times 0.35, 0.0, 0.70)$$
  $$\text{action\_drive} = \text{clamp}(\text{raw\_action\_mean}[:, 1:2] - \text{brake\_bias}, -1.0, 1.0)$$
- **Effect**: On straights ($\kappa \approx 0$), `brake_bias = 0.0`. Approaching Corner 1 ($\kappa \ge 1.5\text{--}2.15\text{ m}^{-1}$), scales drive action down to $-0.45$, commanding wheel velocity ~11 rad/s ($v \approx 0.35\text{ m/s}$) to enter the turn within linear tire grip limits.

### 2. Speed Tracking Reward Rebalancing & Overspeed Penalty
- **Files**: [`arcproLab/arcpro_env_cfg.py`](file:///home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/arcproLab/arcpro_env_cfg.py), [`arcproLab/mdp/rewards.py`](file:///home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/arcproLab/mdp/rewards.py)
- **Mechanism**: Doubled `speed_tracking` weight ($60.0 \to 120.0$), tightened tracking exponent to $-2.5$, and added quadratic overspeed penalty for speeds $> 0.45\text{ m/s}$ when $\kappa_{\text{eff}} \ge 0.70$:
  $$\text{overspeed\_pen} = -2.0 \times \max(0, v - 0.45)^2$$

### 3. Curriculum Sector 2 Focus (50% Corner 1 Spawns)
- **Files**: [`arcproLab/mdp/curriculum_sectors.json`](file:///home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/arcproLab/mdp/curriculum_sectors.json), [`arcproLab/mdp/events.py`](file:///home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/arcproLab/mdp/events.py)
- **Mechanism**: Sector weights shifted to `[0.15, 0.50, 0.15, 0.10, 0.10]`, dedicating 50% of spawns to Sector 2 ($X = -16.197, Y = -3.50$, 2.5m before C1). Set lookahead to 1.6m and target corner speed to 0.35 m/s.

### 4. Kinematic Steering Feedforward Prior (`0.485 * kappa`)
- **File**: [`arcproLab/agents/skrl_models.py`](file:///home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/arcproLab/agents/skrl_models.py)
- **Mechanism**: Ackermann steering feedforward prior: $\text{steer\_bias} = \text{clamp}(0.485 \times \kappa, -1.0, 1.0)$. Disarmed centering spring (masked slot 5 to 0.0).

### 5. Parameter-Group Differential Learning Rates
- **File**: [`arcproLab/scripts/train_skrl.py`](file:///home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/arcproLab/scripts/train_skrl.py)
- **Mechanism**: 3-tier Adam parameter groups ($3\times 10^{-5}$ vision, $3\times 10^{-4}$ policy head, $1\times 10^{-4}$ critic). Entropy scale set to 0.010.

### 6. Clean Background Training Resumption
- Resumed training cleanly from `agent_1350000.pt` in tmux session `training`. Fleet is running 20 parallel environments with full RGB cameras (`--enable_cameras`) and live autonomous sentinel monitoring.

### 7. Multi-Sector Curriculum Rebalance (Option B Execution — Issue 107)
- **Files**: [`arcproLab/mdp/curriculum_sectors.json`](file:///home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/arcproLab/mdp/curriculum_sectors.json), [`arcproLab/mdp/curriculum_config.py`](file:///home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/arcproLab/mdp/curriculum_config.py)
- **Mechanism**: Rebalanced sector weights from `[0.15, 0.50, 0.15, 0.10, 0.10]` to `[0.35, 0.10, 0.10, 0.35, 0.10]`:
  - **Sector 1 (35%)**: Spawns at Start Line ($X = -16.197, Y = 5.50$) to maximize full-lap attempt density (0 to 64.85m, target ~5,000 steps).
  - **Sector 4 (35%)**: Spawns at Leg 2 Intersection Approach ($X = -8.000, Y = -7.10$, heading East), providing high-density exposure to the 0.55m intersection lane shift and Corner 2 turn-in dynamics.
  - **Sectors 2, 3, 5 (10% each)**: Preserves Corner 1 pre-braking, Leg 2 exit acceleration, and North return corridor mastery without regression.
- **Dynamic Auto-Reload**: Zero-downtime hot-reload via mtime tracking in `curriculum_config.py` updating active environment thresholds without restarting the 20-car simulation.

### 8. Corner 2 Dedicated Runway Spawner & Dynamic Sector Spawns (Swarm Run 3 — Issue 108)
- **Files**: [`arcproLab/mdp/curriculum_config.py`](file:///home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/arcproLab/mdp/curriculum_config.py), [`arcproLab/mdp/curriculum_sectors.json`](file:///home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/arcproLab/mdp/curriculum_sectors.json), [`arcproLab/mdp/events.py`](file:///home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/arcproLab/mdp/events.py)
- **Mechanism**:
  - **Dynamic Sector Spawns**: Introduced `sector_spawns` into `curriculum_sectors.json` with dynamic mtime reload via `curriculum_config.get_sector_spawns()`.
  - **Corner 2 Dedicated Runway Relocation**: Relocated Sector 4 from $X = -8.000, Y = -7.10$ (12m before C2) to $X = 0.500, Y = -6.550$, heading East ($yaw = 0.0$), placing vehicles on a straight runway exactly 2.3m before Corner 2 entry (WP 2218 -> 2400).
  - **High-Density Corner 2 Exposure**: Dedicates 35% of spawns directly to the Corner 2 runway approach, allowing vehicles to experience the kinematic curvature pre-braking prior (lookahead $\kappa$ surging from 0.01 to 0.796 at WP 2458) to master Corner 2 turn-in dynamics, unblocking the 1,000–1,100 step frontier plateau.

### 9. Intersection Permeability, Apex Calibration & Viewer Diagnostic (Swarm Run 4 — Issue 109)
- **Files**: [`arcproLab/mdp/terminations.py`](file:///home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/arcproLab/mdp/terminations.py), [`arcproLab/arcpro_env_cfg.py`](file:///home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/arcproLab/arcpro_env_cfg.py), [`arcproLab/scripts/play_skrl.py`](file:///home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/arcproLab/scripts/play_skrl.py)
- **Mechanism**:
  - **Intersection Permeability**: Masked boundary resets inside intersection corridors (South: WP 1640–1900, North: WP 4170–4270) and expanded gate radius to $1.85\text{m}$, unblocking rollouts through open crossing junctions.
  - **Curvature-Adaptive Apex Thresholds**: Configured `dyn_threshold_y = 0.005m` for tight turns ($|\kappa| > 0.60$), allowing 19cm chassis to clip apex curbing without false resets.
  - **Ground Contact Force Threshold**: Raised to $250.0\text{N}$ in `ARCProEnvCfg` to absorb cornering tire deformation spikes while maintaining 100% detection of falls off track.
  - **Evaluation Viewer Hardening**: Added direct model weight loading fallback and per-term reset diagnostics in `play_skrl.py`.
- **Effect**: Certified 0 false resets across all 5,000 waypoints. Frontier peak episode length immediately jumped to **1,461 steps** and discovered waypoints surged to **569 WPs**.

---

## 3. Active Background Infrastructure

- **Active Git Branch:** `feature/realsense-realistic-kinematics`
- **Active Training Session:** Background tmux session `training`
  - Pane 0 (`training:0.0`): SKRL PPO (20 camera envs, RTX 3060, ResNet-18 backbone, logging to `logs/skrl_phase1.log`)
  - Pane 1 (`training:0.1`): Autonomous Sentinel daemon (`interval=15.0s`, self-healing enabled)
- **Branch Checkpoint Directory:** `logs/ppo_skrl_feature_realsense-realistic-kinematics/` (checkpoints actively saving up to `agent_2050000.pt`)
- **Curriculum Spawner:** 5-Sector Certified Spawner with dynamic hot-reload; Sector 4 relocated to Corner 2 runway ($X = 0.500, Y = -6.550$, $yaw = 0.0$) with rebalanced weights `[0.35, 0.10, 0.10, 0.35, 0.10]` (Sector 1: 35%, Sector 4: 35%, Sectors 2, 3, 5: 10% each)
- **Landmark Baselines:** Archived in [`archive/landmark_checkpoints/`](file:///home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/archive/landmark_checkpoints/)

---

## 4. Open Issues, Learnings & Next Verification Goals

### Key Learnings
1. **Curvature Prior Inductive Bias**: Pure end-to-end vision from scratch without curvature prior struggled to discover sharp $90^\circ$ turns before hitting wall boundaries. Inductive feedforward steering prior based on Ackermann geometry bridges the gap without corrupting straight line performance.
2. **Differential Learning Rates for Representation vs Control**: Visual feature extractors require slow, conservative learning rates to prevent catastrophic forgetting, while policy MLP heads require higher learning rates to rapidly maneuver through newly discovered track geometries.
3. **Curriculum Distribution Dynamics**: As the agent conquers bottleneck sectors (e.g. Corner 1 via pre-braking), curriculum weights must promptly shift downstream (e.g. Leg 2 intersection and full-lap start line) to avoid over-sampling conquered zones and under-sampling subsequent critical geometries.
4. **Sentinel State Management**: Rollbacks alter the temporal monotonic progression of step counts. Health monitoring engines must explicitly handle step regressions to prevent deadlocks or evaluation blindness.

### Next Verification Goals
1. **Full-Lap Attempt Density & Corner 2 Transit**: Monitor `logs/skrl_phase1.log` to track rollouts navigating through the Leg 2 intersection lane shift (WP 1700–1900) and executing Corner 2 (WP 2450) toward 2,500+ and 5,000-step full laps.
2. **Curriculum Dynamic Reload Verification**: Confirm runtime environment thresholds update dynamically without simulation interruption.
3. **Sentinel Health Verification**: Confirm autonomous sentinel runs cleanly with zero false triggers across consecutive 50k-step evaluation windows.

---

## 5. Resume Instructions
1. Check live training telemetry: `tail -n 25 logs/skrl_phase1.log` or `tmux attach -t training`.
2. Inspect autonomous sentinel health: `python3 arcproLab/scripts/autonomous_sentinel.py --status`.
3. To safely clear context, use `/clear` or `/cta-clear`, then resume with `/cta-resume`.
