# CTA RESUME HERE

## Active Execution State
- **MILESTONE**: M004
- **Phase**: 02
- **Branch**: `feature/realsense-realistic-kinematics`
- **Task**: Physical Kinematics Unlocking, Intel RealSense D455 Bumper Mounting & Racing Line Slack
- **Next Todo**: Launch warm-start benchmark run on branch from `archive/landmark_checkpoints/agent_4141500_peak.pt` and verify Corner 1 transit.
- **Checkpoint Timestamp**: 2026-09-08T06:24:00+00:00Z
- **Active Git Branch**: `feature/realsense-realistic-kinematics`

## Persistent State Data Sources
- **Turn Actions Database**: `.cta/cta_turns.db`
- **Codebase RAG Database**: `.cta/cta_codebase.db`
- **Codebase Index Guide**: `cta_codebase_index.yml`
- **Directory Structure**: `cta_directory_structure.yml`

## Recent Completed Actions
- **SWARM RESEARCH** [SUCCESS]: Mobilized 5-worker divergent swarm (`run_20260908_061337_0f4201` & `run_20260908_061435_d3314c`), proving Corner 1 kinematic impossibility ($R=0.464\text{m}$ vs $R_{\min}=0.604\text{m}$), discovering PhysX $\pm 35^\circ$ joint clamping bug in `spawner.py`, identifying RealSense D455 global shutter match, discovering floating mid-air camera bug ($25\text{cm}$ in front of bumper), and proving out-in-out racing line requirement.
- **BRANCH CREATION** [SUCCESS]: Committed Step 4.28M baseline to `main` and created dedicated branch `feature/realsense-realistic-kinematics` for clean isolation.
- **KINEMATICS UPGRADE** [SUCCESS]: Raised `spawner.py` RevoluteJoint limits from $\pm 35.0^\circ$ to $\pm 55.0^\circ$ (allowing inner wheel $50.9^\circ$), and raised `max_steering_angle` from $0.50$ to $0.68\text{ rad}$ ($39.0^\circ$), reducing $R_{\min}$ from $0.604\text{m}$ to $0.408\text{m}$.
- **REALSENSE D455 BUMPER MOUNT** [SUCCESS]: Repositioned camera in `arcpro_env_cfg.py` from floating mid-air $(0.5, 0, 0.35)$ down to realistic front bumper $(0.22, 0.0, 0.18)$ with $15^\circ$ pitch. Updated camera marker to match.
- **RACING LINE SLACK** [SUCCESS]: Updated `lateral_error_reward` in `rewards.py` with dynamic curvature slack ($\text{slack} = \text{clamp}(\kappa \times 0.08, 0, 0.10)$). Enforces strict $0\text{cm}$ centering on straights while allowing up to $10\text{cm}$ apex clipping on sharp corners without penalty.
- **BRANCH CHECKPOINT ISOLATION** [SUCCESS]: Updated `train_skrl.py` and `start_tmux_training.sh` to automatically isolate logs and checkpoints into `logs/ppo_skrl_feature_realsense-realistic-kinematics/`, preventing collision with `main`.

## Open Issues, Concerns & Learnings
- **LEARNING** (Kinematic Impossibility of Corner 1 with 0.5 rad Steering): The Corner 1 centerline has $R=0.464\text{m}$. With wheelbase $0.33\text{m}$ and $\delta=0.50\text{ rad}$, the car's minimum geometric turning radius was $R_{\min} = 0.604\text{m}$. Outer clearance was only $6.3\text{cm}$ ($40.6\text{cm}$ road $- 28.0\text{cm}$ car $/ 2$). Thus, following the centerline made crashing mathematically certain ($14\text{cm}$ outward understeer drift).
- **LEARNING** (Hardcoded PhysX Joint Clamping in `spawner.py`): Lines 76–78 in `spawner.py` hardcoded USD RevoluteJoint limits to $\pm 35^\circ$. Inner Ackermann wheel requires $50.9^\circ$ at $\delta=0.68\text{ rad}$. PhysX was clamping the inner wheel, causing tire scrubbing and preventing tight cornering. Raising to $\pm 55^\circ$ unlocks full kinematic travel.
- **LEARNING** (Intel RealSense D455 vs D435i): D435i RGB FOV is only $69.4^\circ$ with a rolling shutter (motion blur). D455 is the true physical match: native $86\text{--}90^\circ$ FOV with Global Shutter. Camera lowered from floating periscope $(0.5, 0, 0.35)$ to realistic front bumper $(0.22, 0, 0.18)$.
- **LEARNING** (Racing Line Dynamics vs Centerline Penalty): Motorsports and autonomous racing use an out-in-out line to expand effective radius to $R_{\text{eff}} \ge 0.70\text{m}$. The linear `lateral_error` penalty was fighting this geometry. Curvature-gated slack allows apex cutting while maintaining straightaway tracking.
- **DECISION** (Warm Transfer from Landmark Peak vs Cold Restart): Rejecting Step 0 cold restart avoids burning 86M transitions and re-entering donut/parking local minima. Resuming from landmark peak `agent_4141500_peak.pt` immediately leverages the extra steering authority on saturated cornering actions.

## Resume Instructions
1. Reset or clear the screen safely using `/clear` or `/cta-clear`.
2. On fresh context, invoke `/cta-resume` to restore project continuity directly from this file and SQLite databases.

