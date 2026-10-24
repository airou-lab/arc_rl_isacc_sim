# RESUME — Context for Next Agent Session
**Last Updated:** 2026-09-08 (Sections 3–4 updated 2026-10-23)
**Active Branch:** `feature/realsense-realistic-kinematics`

> **2026-10-23 NOTE:** Sections 1–2 are historical (old 453k-parameter dense architecture). The live state is in `RESUME HERE.md` at the repo root. Checkpoints from before 2026-10-21 (e.g. `agent_3587500_c1_champion.pt`, `agent_3687500.pt`, `agent_4141500_peak.pt`) **cannot be loaded** into the current 21k Spatial Softmax model (state-dict size mismatch, see Issue 118).

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

### Issue 105: Vision-Driven Lane Boundary Remediation, Checkpoint Pruning & RealSense D435i Alignment
- **Problem**: 
  1. Policy relied on synthetic GPS waypoints for progress rewards (+7.6 pts/step), dwarfing the -10 boundary penalty. Crashing after sprinting down straightaways was net-positive, so the policy never learned white/yellow lines were impassable walls.
  2. Intersection gate routing was missing on Leg 2 (`X < -3.5`), and random mission tokens caused the car to command right (+1.0) into the grass barrier.
  3. Stale 6.19M checkpoints in `logs/` confused automated checkpoint detection.
- **Resolution**:
  1. **Deterministic Kinematic Verification**: `tests/test_manual_left_turn.py` successfully completed the 90-degree left turn at Corner 2 with 13.9cm white / 21.2cm yellow line clearance (zero collisions). Proved physical vehicle, steering, and electronic diff can easily navigate sharp turns.
  2. **Storage Pruning**: Deleted 15 stale and stiff checkpoints across 6 dead runs in `logs/ppo_skrl_rollback_clean-baseline/`.
  3. **Intersection Gate Routing**: Fixed `arcproLab/mdp/road_manager.py` so Leg 2 approaches explicitly command `turn_token = -1.0` (LEFT) into Corner 2 and zero out random mission exploration.
  4. **Reward Restructuring for Pure Vision Lane Following**:
     - `termination_penalty`: Raised to 400.0 (crashing now wipes out all sprint profits).
     - `boundary_penalty`: Raised to 60.0 (continuous push away from white/yellow lines).
     - `lane_centering_reward`: Added dense reward (`exp(-12.0 * |dist_y - dist_w|)`) with weight 120.0 to strongly reward staying equidistant between lines.
     - `progress_reward`: Scaled down to 80.0 so safety/boundary avoidance dominates.
     - MDP braking advantage increased from +0.709 to **+1.410 pts/step (+70.5 pts/sec)**.
  5. **Camera Grounding**: Validated RealSense D435i geometry (15 deg downward pitch at 0.18m bumper mount covers road 0.25m to 4.0m ahead).

### Issue 106: Intersection Teacher Guidance, Turn Penalty Elimination & Curriculum Focus
- **Problem:**
  1. At the intersection, painted lines break for cross traffic. The agent lacked visual lines to center between and steered sluggishly into the oncoming lane.
  2. `action_steer_penalty` (-0.5) and high `jerk`/`smoothness` penalties directly penalized steering into curves.
  3. `action_drive_reward` (+10.0) bribed the policy to sprint straight at full throttle.
  4. After the intersection, the sweeping road bend ($Y=1.5\text{m} \to 7.1\text{m}$) caused cars to drive off because `turn_token` was reset to 0.0 (Straight) and steering penalties fought curve tracking.
- **Resolution (CTA Swarm `run_20261001_001746_6ca943` - 3/3 PASSED):**
  1. **Intersection Teacher Guidance**: Added `intersection_expert_reward` inside `rewards.py` and `arcpro_env_cfg.py` (weight 100.0) providing geometric Stanley steering and 0.35 m/s speed targets inside intersection zones.
  2. **Penalty Elimination**: Removed `action_steer_penalty` and `action_drive_reward`; relaxed `jerk` (0.05 -> 0.01) and `smoothness` (2.0 -> 0.5) so the car can steer sharply into corners without penalty.
  3. **Full-Circuit Left Turn Routing**: Updated `road_manager.py` to route `turn_token = -1.0` (LEFT) for Corner 1, Corner 2 / Intersection, the sweeping Road Bend, and Corner 4.
  4. **Intersection Curriculum Immersion**: Updated Sector 4 in `events.py` to spawn at $X=-6.0\text{m}, Y=-7.118\text{m}$ (0.8m before turn entry) with 40% curriculum weighting.
  5. **Kinematic & Regression Verification**: `tests/test_manual_left_turn.py` completed 350-step transit with 13.9cm white / 21.0cm yellow line clearances; regression tests passed 4/4.

### Issue 107: Intersection Permeability, Stop-Bar Punching & Early Termination Remediation
- **Problem:**
  1. At Sector 4 intersection entry ($X=-5.0\text{m}$), 582 painted stop-bar white points crossed the lane, triggering false `dist_w < 0.04m` boundary resets.
  2. Initiating the left turn caused the car to straddle/touch the double yellow line, triggering `dist_y < 0.02m` resets because `is_corner` misclassified the turn ($\kappa = 0.24 < 0.35$).
  3. Corner 2 had only 1 gate fallback point `[-3.85, -6.50]`, leaving $X \in [-6.0, -4.5]$ with `dist_g > 0.50m` (unmasked).
- **Resolution (CTA Swarm `run_20261001_042016_c3d3e3` - 4/4 PASSED):**
  1. **Boundaries (`track_boundaries_1x.npz`)**: Punched 582 stop-bar white points and 676 yellow crossing points in Corner 2 corridor; added 50 dense 3D gate points along turn centerline ($X \in [-6.3, -2.5], Y \in [-7.1, -4.2]$).
  2. **Terminations (`terminations.py`)**: Added `in_intersection` geometric check (`X in [-6.5, -2.0], Y in [-7.5, -3.8]`), lowered `is_corner` threshold to $\kappa > 0.20$, and permitted painted line crossings inside intersection while enforcing envelope limit ($|lat\_err| < 0.70\text{m}$).
  3. **Rewards (`rewards.py`)**: Masked `boundary_penalty` inside intersection and extended `intersection_expert_reward` zone from spawn entry ($X=-6.5\text{m}$).
  4. **Verification**: `test_cornering_remediation.py` passed 4/4; `test_manual_left_turn.py` completed 350-step PhysX transit with $14.6\text{cm}$ white / $22.8\text{cm}$ yellow clearance.

### Issue 108: Intersection Pre-Braking Alignment & Understeer Washout Resolution
- **Problem:**
  1. In `./debug.sh --sector 4`, the car survived past the stop bar into the intersection (reaching Step 193), but crashed into the outer white curb.
  2. The policy charged into the 1.2m radius corner at **0.85 – 0.90 m/s**, causing massive centrifugal understeer ($a_c \approx 0.68\text{ g}$) even under full left steering lock (`Steer: +0.66`).
  3. `curvature_speed_reward` (weight 180.0) commanded $0.68\text{--}0.85\text{ m/s}$ based on turn curvature $\kappa = 0.24$, conflicting with and overpowering `intersection_expert_reward` (weight 100.0) which commanded $0.35\text{ m/s}$.
- **Resolution:**
  1. **Speed Target Alignment (`rewards.py`)**: Enforced a strict $0.35\text{ m/s}$ target corner speed inside the intersection approach corridor ($X \in [-7.0, -2.0], Y \in [-7.5, -3.8]$ or `dist_g < 1.50m`), providing +213.8 pts/step reward incentive to brake before the turn.
  2. **Kinematic Verification (`test_manual_left_turn.py`)**: Confirmed 350-step PhysX transit through the intersection at $0.35\text{--}0.40\text{ m/s}$ with $14.6\text{ cm}$ white and $23.5\text{ cm}$ yellow clearances.
  3. **Resumption without Rollback**: Warm-started training cleanly from `agent_3337500.pt` (all-time peak baseline) on 20 camera envs in tmux session `training` with autonomous sentinel active.

### Issue 109: Runway Premature Turn Token Remediation, Curriculum Balancing & Clean Baseline Rollback
- **Problem:**
  1. In `./debug.sh`, the vehicle in Sector 2 (Corner 1 Runway Approach) steered hard left (`Steer: +0.72 to +0.88`), straddled/crossed the double yellow line (`Yellow: 2.2cm`), and locked its brakes to a dead stop ($0.00\text{ m/s}$) from step 140 to 240.
  2. Multi-checkpoint headless evaluation (`tests/compare_corner1_checkpoints.py`) confirmed that overnight model `agent_3437500.pt` suffered from a destructive left-steering bias on straight runways, crashing into the yellow line at Step 90 (`Yellow: 2.3cm`).
  3. Meanwhile, baseline `agent_3337500.pt` drove cleanly with centered steering (`-0.04 to +0.03`, `White: 18.5cm`, `Yellow: 22.2cm`), traversed 312 waypoints, and survived 301 steps without incident.
- **Root Cause:**
  1. **Premature Turn Token in `road_manager.py`**: `in_c1_turn` was triggered at $Y < -3.5\text{ m}$ (and `in_c4_turn` at $X < -12.0\text{ m}$), setting `turn_token = -1.0` 2.0 to 2.5 meters before curves actually begin.
  2. **Curriculum Over-Immersion**: 90% of training spawns were in or approaching left turns (40% Sector 4, 25% Sector 2, 10% Sector 3, 15% Sector 5), teaching the actor MLP to immediately crank left (+0.80 to +0.95) whenever `turn_token = -1.0`.
  3. **Yellow Line Braking Lockup**: Once touching the yellow line, `too_close_y` penalized driving forward by -420 pts/sec, while sitting still cost only -40 pts/sec. The car parked because it was mathematically optimal once straddling the line.
- **Resolution:**
  1. **Model Rollback**: Quarantined corrupted overnight run `20261001-212131` to `corrupted_20261001-212131_left_bias`. Active checkpoint automatically resolved to verified clean baseline `agent_3337500.pt`.
  2. **Bounding Box Tightening (`road_manager.py`)**: Adjusted turn token activation so straight runways broadcast `turn_token = 0.0` (Lane Follow) and only trigger `turn_token = -1.0` when curves actually begin ($Y < -5.5$ for C1, $X > -5.2$ for Intersection, $X < -14.6$ for C4).
  3. **Curriculum Balancing (`curriculum_sectors.json`)**: Rebalanced sector weights to a uniform 20% across all 5 sectors (`[0.20, 0.20, 0.20, 0.20, 0.20]`).
  4. **Sentinel Threshold (`autonomous_sentinel.py`)**: Adjusted `CRAWLING_TRAP` threshold from $0.40\text{ m/s} \to 0.25\text{ m/s}$ to accommodate healthy corner pre-braking targets ($0.35\text{ m/s}$).

### Issue 110: 20-Environment Camera VRAM Leak Elimination & Detached Sentinel Self-Healing
- **Problem:**
  1. During 20-camera environment training (`--enable_cameras`), GPU VRAM grew monotonically by ~5.56 MB per step (~1,112 MB per 200 steps). After ~6,000 steps, VRAM exceeded 12 GB, triggering fatal `torch.OutOfMemoryError` and killing Isaac Sim.
  2. When the Autonomous Sentinel attempted to self-heal and restart training via `subprocess.run(["./start_tmux_training.sh"])`, the script executed `tmux kill-session -t training`. Because the Sentinel was running inside a tmux pane sharing the server, killing the session killed the Sentinel daemon itself, causing a restart suicide.
- **Root Cause:**
  1. **ObservationManager Sensor Cloning:** In `arcpro_env_cfg.py`, `tiled_camera = ObsTerm(func=mdp.image, ...)` caused IsaacLab's `ObservationManager.compute_group` to invoke `.clone()` on line 395 and cache the $20 \times 224 \times 224 \times 3$ uint8 tensor (2.87 MiB) in `ObservationManager._obs_buffer` and `base_env.obs_buf` every step. Execution frames and internal dictionary buffers retained references to previous steps across simulation iterations.
  2. **Dynamic Tensor Concatenation:** In `arcproLab/agents/skrl_wrappers.py`, `torch.cat([visual_flat, vec], dim=1)` dynamically allocated a new $20 \times 25,100$ float32 tensor (~1.95 MiB) on every single step.
  3. **Tmux Process Hierarchy:** The Sentinel executed restart commands synchronously within its own tmux session group without POSIX session detachment.
- **Resolution (CTA Swarm `run_20261007_150359_1a3126` - 2/2 PASSED):**
  1. **Direct Static Sensor Reading (`arcpro_env_cfg.py`, `skrl_wrappers.py`):** Removed `tiled_camera` from `ObservationCfg.PolicyCfg` in `arcpro_env_cfg.py`. Updated `SKRLFlattenWrapper._flatten_obs()` to read directly from Isaac Sim's static GPU texture buffer: `base_env.scene.sensors["tiled_camera"].data.output["rgb"]`. This completely eliminated all tensor cloning and dictionary caching inside `ObservationManager`.
  2. **Preallocated Observation Output Buffer (`skrl_wrappers.py`):** Preallocated a single static tensor buffer `self._flat_obs_buffer` of shape `(num_envs, 25100)` in `SKRLFlattenWrapper` and updated it in-place using `.copy_()`, completely eliminating dynamic `torch.cat` allocations.
  3. **Detached Sentinel Process Isolation (`autonomous_sentinel.py`, `start_tmux_training.sh`):** Replaced synchronous restarts with `subprocess.Popen(..., start_new_session=True, env=clean_env)` with `RUNNING_DETACHED=1` and stripped `TMUX` variables, ensuring the restarter survives `tmux kill-session`.
  4. **Adversarial Gate Verification (`tests/test_vram_leak_gate.py`):** Ran 200 adversarial test steps across 20 RGB environments. VRAM delta dropped from **+1,112.82 MB down to +4.11 MB (<0.02 MB/step)**, passing the gate with exit code 0.
  5. **Training Resumption:** Cleanly resumed 20-env fleet training under Sentinel from `agent_3337500.pt` with stable VRAM at 5.39 GB / 12.28 GB (>6.8 GB free headroom).

### Issue 111: Straightaway Over-Speed Damping, Rightward Drift Remediation & Champion Rollback
- **Problem:** Max episode length capped at 872 steps (only 8.72m of 50.72m circuit). Policy developed straightaway over-speeding ($0.94\text{ m/s}$) and rightward drift bias ($-22\text{ cm}$), clipping outer white curb.
- **Resolution (CTA Swarm `run_20261009_004524_1cf812` - 2/2 PASSED):**
  1. Quarantined `20261008-150421` to `corrupted_20261008-150421_right_bias`.
  2. Immortalized champion `agent_3587500.pt` to `archive/landmark_checkpoints/agent_3587500_c1_champion.pt` (verified 100% survivor on Corner 1, 400 steps, 605 waypoints, 12.8cm white / 21.0cm yellow clearance).
  3. Stabilized straightaway target speed in `rewards.py` from 0.85 m/s to 0.70 m/s (`target_speed = torch.clamp(0.70 - effective_kappa * 0.50, min=0.35, max=0.70)`).
  4. Updated `autonomous_sentinel.py` to ignore `corrupted` directories and prioritize newest `agent_*.pt`.
  5. Regression test suite (`tests/test_cornering_remediation.py`) PASSED 4/4.
  6. Warm-started fleet training from `agent_3587500.pt` with active Sentinel supervision.

### Issue 112: Intersection Approach Permeability, Turn Token Anticipation & Asymmetric Curvature Slack
- **Problem:**
  1. Curve outer line straddling: robot riding/straddling outer white line with wheels over it.
  2. Straightaway false termination: when approaching intersection on Leg 2 ($Y = -7.118$), robot terminated before reaching intersection for no reason.
  3. Intersection turn washout: at intersection start (Sector 4), robot failed to steer tightly enough into the 90-degree left turn, drifting into the eastern grass cliff.
- **Root Cause & Resolution:**
  1. **Straightaway False Termination**: Leg 2 gate points start at $X = -6.32\text{ m}$. In `terminations.py`, `in_permissive_box` was gated at $X \ge -6.2\text{ m}$, leaving an 80cm unpermeable zone ($X \in [-7.0, -6.2]$) where approaching stop markings triggered hair-trigger resets. Expanded `in_permissive_box` to $X \in [-7.0, -2.15], Y \in [-7.5, -2.4]$ and added explicit gate proximity permeability `dist_g < 0.60\text{ m}` with $|lat\_err| \le 0.40\text{ m}$ corridor and east cliff guard ($X > -2.08\text{ m}$).
  2. **Intersection Turn Washout**: In `road_manager.py`, `in_c2_turn` previously activated late ($X > -5.2\text{ m}$) and prematurely ended ($Y \le -5.5\text{ m}$). Updated `in_c2_turn` to activate at $X > -6.8\text{ m}$ and persist through $Y \le -2.4\text{ m}$, providing early turn anticipation and continuous left steering into the Northbound straight.
  3. **Curve White Line Straddling**: Symmetric curvature slack allowed up to 10cm outward drift. Restricted slack to inward apex-cutting ($lat\_err > 0$, max 3cm) while outward drift ($lat\_err \le 0$) has strict 0cm slack. Advanced white line boundary penalty warning distance from 16cm to 18cm (penalizing tire proximity within 4cm).
  4. **Kinematic Verification**: `tests/test_manual_left_turn.py` completed 350-step transit with 14.6cm white / 21.7cm yellow clearance.

---

## 3. Active Background Infrastructure (as of 2026-10-23)

- **Architecture:** 21,476-param Spatial Softmax actor (Issue 117). Fresh start 2026-10-21 16:33 (Issue 118).
- **Training Session:** tmux `training` — pane 0.0 `train_skrl.py --num_envs 48 --headless --enable_cameras`, pane 0.1 `autonomous_sentinel.py --daemon --interval 30.0`. Target: 2M steps.
- **Active Run:** `logs/ppo_skrl_rollback_clean-baseline/20261022-192651/` (step ~366k at 11:13 2026-10-23). Sentinel auto-restarts on VRAM > 11.5 GB (~every 12–14h) and resumes from the newest `agent_*.pt`, so a newer run dir may exist.
- **Landmark Baselines:** Everything in [`archive/landmark_checkpoints/`](file:///home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/archive/landmark_checkpoints/) is 453k-architecture and **incompatible** with the current model. Do not resume from it.

---

## 4. Next Steps for Incoming Agent / User

1. **Check training:** `tail logs/skrl_phase1.log`, `tail logs/sentinel.log`, `nvidia-smi`. Do not restart unless it is down; if needed, `./start_tmux_training.sh` with no args auto-resumes from the newest checkpoint.
2. **Open concern (cs_turns concern #35):** the deterministic policy does not turn at Sector 4. Probing the policy head with turn_token (input index 64) = -1.0 moves steering only +0.003..+0.006 rad (200k–350k). Re-probe each new checkpoint; if still ~0 by ~500k, investigate turn_token reward/obs coupling before training further.
3. **GUI check:** `./debug.sh --sector 4` needs ~2.3 GB VRAM. Stop training first if VRAM is above ~9 GB.

