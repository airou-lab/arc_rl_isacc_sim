# CTA RESUME HERE

## Active Execution State
- **MILESTONE**: M004: RealSense Realistic Kinematics & Clean Step 0 Bootstrap
- **Phase**: Phase 03: Step 0 Scratch Training & Corner 1 Transit
- **Task**: Repository Cleanup & Milestone 250k Checkpoint Monitor
- **Next Todo**: Continue monitoring training fleet past Step 250k towards Corner 1 transit (>860 steps). Checkpoint agent_250000.pt is secured on disk.
- **Checkpoint Timestamp**: 2026-09-11T17:21:19.318634+00:00Z
- **Git Commit**: 440614ef38af92cf40e0b93c4618671b89114a1d

## Persistent State Data Sources
- **Turn Actions Database**: `.cta/cta_turns.db`
- **Codebase RAG Database**: `.cta/cta_codebase.db`
- **Codebase Index Guide**: `cta_codebase_index.yml`
- **Directory Structure**: `cta_directory_structure.yml`

## Recent Completed Actions
- **REFACTOR** [SUCCESS]: Completed full repo cleanup, reclaimed 25.5GB disk, pruned dead scripts, and styled codebase
- **INVESTIGATION** [SUCCESS]: Repository cleanup, code style, and simulation safety audit
- **EXECUTION** [SUCCESS]: Audit Repository Clutter, Dead Files & Cache Bloat

## Open Issues, Concerns & Learnings
- **LEARNING** (Sentinel Safeguards Audit for Early Learning): 1. Sentinel 3 ('WALL_HUGGING') at line 426 triggered repeatedly (9 times between 22:04 and 00:07) because it lacks an activation step gate. A newborn policy at step 50k naturally exhibits avg_ep_len ~135-145 steps while learning to navigate curves from the 5-sector spawner (especially Sector 2 with 2.5m runway to C1). With avg_speed ~0.62 m/s and avg_rew ~540, it satisfied all 3 conditions: ep_len < 150, speed >= 0.50, rew < 600. However, telemetry confirms the policy was healthy and actively advancing (wps_cum=50-78, max_len=370, reaching 597 steps and 1010 rew by step 58k). Sentinel 3 MUST be gated with latest_batch.step >= 100_000 or 150_000 and should also check wps_cum < 10 to prevent false-positive restarts of newborn exploration. 2. Sentinel 1 (Crawling) is safe due to ep_len >= 150 gate. Sentinel 2 (Apex Cliff) is well-gated with step >= 300,000. Sentinel 4 (Collapse) and Sentinel 5 (Hang) are properly guarded and functioning as intended. 3. Training survived Sentinel 3 interventions due to tmux session self-guarding in start_tmux_training.sh preventing accidental session termination. 4. arcproLab/__init__.py top-level imports prevent importing AutonomousSentinel outside the Omniverse environment unless mocked.
- **LEARNING** (PPO Convergence & Checkpointing Cadence Audit): 1. Learning Rate (3e-5): Set for fine-tuning, slightly conservative for training from scratch, but exceptionally stable given the frozen ResNet-18 vision backbone. Successfully drove Step 58k breakthrough (Rew: 1010.0, Len: 229 mean / 597 max, Spd: 0.60 m/s, WPs: 99). Entropy scale 0.005 + KL threshold 0.016 provides balanced exploration without policy collapse. 2. Checkpoint Interval (250k steps): Unacceptably slow (~12.1 hrs between saves at ~20.7k steps/hr); currently 0 checkpoints saved at step 58k. Highly recommend lowering to 50,000 steps (~2.4 hrs, ~630MB total over 5M steps) or 100,000 steps (~4.8 hrs). External ptrace checkpoint triggering on live Isaac Sim process is unsafe due to Carb/Vulkan deadlock risks; recommend adding file-watch hook for safe on-demand checkpoints. 3. Memory & Batching: 1024 rollouts * 20 envs = 20,480 samples / 8 mini-batches = 2,560 transitions/batch. Memory footprint is ~86MB VRAM, providing stable low-variance gradients.
- **LEARNING** (Step 0-58k Learning Trajectory Analysis): Step 0-58k Telemetry & Learning Trajectory Analysis for feature/realsense-realistic-kinematics:
1. Episode Length Trajectory:
   - 0-15k: Mean len=145.93, Avg max len=259.2, Peak len=375.0 (0 batches >= 400)
   - 15k-35k: Mean len=141.18, Avg max len=276.8, Peak len=568.0 (4 batches >= 400: Steps 24999 [424], 28999 [568], 32999 [440], 34499 [407])
   - 35k-58k: Mean len=148.39 (surging to 229.0 at step 57999), Avg max len=297.5, Peak len=597.0 in SKRL log / 639.0 in TensorBoard (4 batches >= 400: Steps 43499 [474], 44999 [451], 57499 [426], 57999 [597])
   - Peak episode length has decisively broken past 400 steps, achieving all-time run peak of 597 steps (SKRL) / 639 steps (TensorBoard).

2. Speed Trajectory:
   - 0-15k: Mean speed = 0.510 m/s
   - 15k-35k: Mean speed = 0.565 m/s
   - 35k-58k: Mean speed = 0.593 m/s (hitting 0.60 m/s at step 57999)
   - Overall fleet speed: 0.562 m/s. The fleet reliably maintains the healthy 0.55 - 0.75 m/s forward corridor without stalling (<0.40 m/s) or runaway sprinting (>1.0 m/s).

3. Waypoint Navigation:
   - WPs_cum progression: 0-15k avg = 52.87 WPs; 15k-35k avg = 54.40 WPs; 35k-58k avg = 64.13 WPs. Max WPs reached 116.0 at Step 56999 (TB max 125.45 WPs), representing >2x increase in waypoint discovery.
   - WPΔ_rew progression: Steered steadily from 0.74 (0-15k) to 0.83 (15k-35k) and 0.86 (35k-58k), peaking at 1.35.

4. Reward Trajectory:
   - Mean reward: 518.15 (0-15k) -> 522.55 (15k-35k) -> 574.53 (35k-58k), surging to +1,010.0 at Step 57999.
   - Peak reward: 1,904.6 (0-15k) -> 3,203.9 (15k-35k) -> 3,396.1 (35k-58k, TB max 3,624.5).

5. Lane Centering & Sensor Telemetry:
   - Dist_White_m: Mean = 0.2046 m, latest = 0.1959 m.
   - Dist_Yellow_m: Mean = 0.2082 m, latest = 0.2179 m.
   - Perfect lane centering across track width (delta < 4 mm).

6. Health Assessment vs Historical Failure Modes:
   - Donut farming / spinning top: Completely absent; WPs_cum averages 58-64 WPs with continuous forward speed.
   - Crawling / parking trap: Absent; Sentinel 1 never triggered.
   - Wall-hugging / suicide loops: Sentinel 3 flagged mild wall hugging at steps 10k, 16k, and 52k, automatically refreshed symmetry and self-healed.
   - Visual lane tracking: Policy demonstrates genuine visual lane tracking from birth with RealSense D455 bumper camera geometry.
- **LEARNING** (Vision & Observation Audit for Step 0 Restart): Systematic audit of vision backbone and AAC observation pipeline: 1. Camera positioning at pos=(0.22, 0.0, 0.18) with 15 deg pitch and 90 deg FOV covers ground from 10.4 cm in front of camera (7.4 cm ahead of bumper), successfully capturing left/right lane boundaries. 2. SKRLFlattenWrapper uses frozen ResNet-18 layer4 yielding (512, 7, 7) = 25,088 dims, resulting in 25,100-dim observation space. This preserves spatial left/right geometry but consumes ~2.1-4.2 GB VRAM in rollout buffer. 3. CRITICAL FLAW: IsaacLabWrapper.step() discards the 'critic' observation group and returns only 'policy'. Consequently, ARCProCritic slicing state[:, -12:] receives the masked policy telemetry (lat_err=0, head_err=0, dist=0) rather than privileged ground-truth state. 4. PyTorch pipeline has zero CPU-GPU bottlenecks (100% CUDA resident).
- **LEARNING** (Kinematics & Actuators Audit for Step 0 Restart): Kinematics and Physics Actuator Audit Summary for Step 0 Restart:
1. Steering Limits & Ackermann Geometry:
- USD RevoluteJoint limits in spawner.py (_apply_steering_limits) are verified at +/-55.0 deg (+/-0.96 rad) on Joint_Steer_L and Joint_Steer_R.
- Ackermann kinematics at max action (+/-1.0): max_steering_angle=0.68 rad (38.96 deg), wheelbase L=0.33m, track width W=0.28m yields turning radius R_min=0.4081m (safely below Corner 1 R=0.464m).
- At max lock (+/-1.0 action), inner wheel angle reaches 0.8886 rad (50.91 deg) and outer wheel reaches 0.5420 rad (31.05 deg). Both remain safely inside the PhysX +/-55.0 deg limits with a 4.09 deg margin, preventing joint clipping.
2. Drive Action Mapping & Orientation:
- GroupedJointVelocityAction in arcpro_env_cfg.py configures scale=20.0, offset=20.0 for Joint_Drive_.*, mapping action [-1, 1] to [0.0, 40.0] rad/s (0.0 rad/s at -1.0, 20.0 rad/s cruise at 0.0, 40.0 rad/s sprint at +1.0).
- Root physics body X points to rear wing; wheel joints are oriented such that positive velocity (+rad/s) pushes vehicle forward towards nose/camera. In observations.py, yaw is corrected by +pi and local X velocity is negated for distance accumulation.
- Actuators: throttle actuator in arcpro_robot_cfg.py provides AWD drive (effort_limit_sim=1.0 per wheel, 4.0 Nm total), damping=1.0, armature=0.01.
3. Tire Friction & Ground Interaction:
- In spawner.py (_apply_tire_friction), static=3.0, dynamic=3.0, restitution=0.0. While higher than standard rubber-on-asphalt (1.0-1.2 / 0.8), this high traction prevents lateral slip on Corner 1 apex without causing instability. Restitution 0.0 eliminates bounce.
4. Center of Mass & Flip/Wheelie Immunity:
- Total vehicle mass is 4.092 kg (chassis 3.342 kg, 4 wheels 0.60 kg, knuckles 0.15 kg).
- Chassis CoM offset is lowered by 0.01m (1cm) in Z.
- Gravity restoring pitch torque (6.62 Nm) exceeds maximum tractive pitch torque (1.93 Nm) by 3.4x. Lateral rollover threshold is 3.5g (max cornering accel is 0.16g). The vehicle is fully immune to wheelies or rollover.
5. Verification: ARCPRO_ROBOT_CFG verified valid in IsaacLab runtime.

## Resume Instructions
1. Reset or clear the screen safely using `/clear` or `/cta-clear`.
2. On fresh context, invoke `/cta-resume` to restore project continuity directly from this file and SQLite databases.
