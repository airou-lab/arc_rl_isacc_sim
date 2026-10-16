# CLEARSET RESUME HERE

## Active Execution State
- **MILESTONE**: M004: Pure Vision Closed-Loop Lap Navigation
- **Phase**: Phase 4.3: Balanced Full-Circuit Curriculum & Physical Boundary Robustness
- **Task**: Task 4.3.6: Sector 4 Geometry Relocation, Height Termination & Champion Rollback
- **Next Todo**: User visual inspection via ./debug.sh --sector 3 / ./debug.sh --sector 4; resume fleet training from agent_3587500_c1_champion.pt.
- **Checkpoint Timestamp**: 2026-10-15T02:37:00.000000+00:00Z
- **Git Commit**: 56f7644876c09fe7412b600fc2c50e0a524f5276

## Persistent State Data Sources
- **Turn Actions Database**: `.cta/cta_turns.db`
- **Codebase RAG Database**: `.cta/cta_codebase.db`
- **Codebase Index Guide**: `cs_codebase_index.yml`
- **Directory Structure**: `cs_directory_structure.yml`

## Recent Completed Actions
- **ROLLBACK** [SUCCESS]: Executed Option B rollback; quarantined Oct 13-14 runs into archive/corrupted_void_spawns_20261013-14/ and set active checkpoint to certified landmark champion agent_3587500_c1_champion.pt
- **VERIFICATION** [SUCCESS]: Single-env simulation verified agent_3587500_c1_champion.pt stepped 30 iterations with zero errors
- **FIX** [SUCCESS]: Relocated Sector 4 from unmeshed void (-2.335, -0.500) to solid asphalt at Intersection Approach (-6.000, -7.118, yaw=0.0) in events.py
- **FIX** [SUCCESS]: Implemented height_termination in terminations.py and registered height_bounds in arcpro_env_cfg.py (Z < -0.05m or Z > 0.50m triggers instant reset)
- **VERIFICATION** [SUCCESS]: 20-env simulation audit confirmed 0/20 underground envs (Z in [0.077m, 0.118m]) and verified hard reset trigger when forced below ground
- **AUDIT** [SUCCESS]: 150-step simulation audit on agent_3937500.pt achieved 100% survival rate (5/5 envs) with healthy line clearances (18-19cm white)

## Open Issues, Concerns & Learnings
- **LEARNING** (Deterministic Policy Survival vs Stochastic Exploration Lengths): Strict eval-mode benchmarking on agent_3937500.pt confirmed the policy genuinely navigates 553 steps and 645 waypoints at 0.63 m/s (matching the rolling average Len 350-550 and WPs 600-690 reported in training). The outlier 1,000–1,600 step runs seen in live training benefit from Gaussian exploration noise nudging the vehicle away from shoulders; in deterministic mode, a slight straightaway drift bias accumulates over ~100 steps until the outer white line is touched on the West straight.
- **LEARNING** (Straightaway Sprint Drift vs Cornering Mechanics Root Cause): Side-by-side benchmarking proved the policy had already learned Corner 2 intersection mechanics and navigates it cleanly with 0 collisions. The failure mode in Sector 3 audits was high-speed straightaway sprinting (0.92 m/s) on narrow 44cm lanes, accumulating +16.9cm leftward lateral error and clipping the double yellow line 2.4m before the turn entry. Lowering the cruising clamp to 0.65 m/s, increasing overspeed penalty to 3.5x, and bolstering yellow line margin to 19cm resolves this failure mode.
- **LEARNING** (Intersection Corridor Envelope and Turn Token Prompt Cutoff): Proved that Sector 4 was prematurely terminated at step 369 because of a rigid 0.40m envelope limit in an 8m open intersection despite healthy line clearances. Furthermore, Corner 1 and Corner 4 turn tokens lingered 1.7m onto straightaways, commanding left turns into the double yellow line. Tightening shutoffs and expanding intersection corridor to 0.55m completely resolves these edge-case terminations.
- **LEARNING** (Leg 2 Yellow Line Proximity Drift and Corner Pre-Braking Resolution): Heavy invasive audit revealed that asymmetrical boundary thresholds (16cm yellow vs 18cm white) encouraged yellow line hugging on Leg 2, while sub-linear speed tracking allowed 0.95 m/s sprinting into 1.2m radius turns. Symmetrizing thresholds to 18cm and adding corner overspeed suppression completely aligns braking and lane centering incentives.
- **LEARNING** (Corner 1 & Corner 4 Centerline Waypoint Defect Resolution): Deep waypoint audit revealed that Corner 1 and Corner 4 previously used naive elliptical arcs with R=1.7m that crossed into the oncoming traffic lane and clipped directly on top of the double yellow line (dist_y = 0.09cm at WP 1254). Following the centerline forced the 28cm car to straddle the yellow line and trigger instant resets. Re-centered the arcs using radius 0.50m centered at (-15.705, -6.60) and (-15.705, 6.60), ensuring minimum clearance to both white and yellow lines is >= 17.7cm everywhere on the circuit (20.0cm yellow / 29.9cm white in C1).
- **ISSUE** (Intersection Gate Box Discrepancy & Turn Token Sequence): Gate points begin at X=-6.32m on Leg 2. Setting in_permissive_box at X>=-6.2m left an 80cm gap (X in [-7.0, -6.2]) where the vehicle approaches the intersection stop markings with dist_g active but without gate permeability, causing instant straightaway termination before reaching intersection. In addition, turn_token=-1.0 must activate early at X>-6.8m and persist through Y<=-2.4m so the policy executes the 4.1m radius turn into the Northbound straight without washing out into the east cliff (X>-2.08m).

## Resume Instructions
1. Reset or clear the screen safely using `/clear` or `/cs-clear`.
2. On fresh context, invoke `/cs-resume` to restore project continuity directly from this file and SQLite databases.
