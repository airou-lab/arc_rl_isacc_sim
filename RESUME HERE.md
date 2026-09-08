# CTA RESUME HERE

## Active Execution State
- **MILESTONE**: M004
- **Phase**: 01
- **Task**: Deploy 5-Sector Spawner, Sentinel, and 250k Interval
- **Next Todo**: Monitor training rollouts past Step 4.5M towards 5.0M with 250k checkpoint intervals and autonomous sentinel active.
- **Checkpoint Timestamp**: 2026-09-08T05:57:59.264033+00:00Z
- **Git Commit**: af8b7278eb80932e758ba26aeb197f1dc653c849

## Persistent State Data Sources
- **Turn Actions Database**: `.cta/cta_turns.db`
- **Codebase RAG Database**: `.cta/cta_codebase.db`
- **Codebase Index Guide**: `cta_codebase_index.yml`
- **Directory Structure**: `cta_directory_structure.yml`

## Recent Completed Actions
- **REFACTOR** [SUCCESS]: Backed up latest checkpoint and landmarks, pruned 1230+ intermediate checkpoints freeing 7.4GB, and restarted training with 250k checkpoint interval
- **REFACTOR** [SUCCESS]: Updated checkpoint_interval from 500 to 250000 (every 0.25M steps) in train_skrl.py to prevent disk runaway
- **EXECUTION** [SUCCESS]: Implemented and deployed autonomous self-healing sentinel with 5 failure mode sentinels, dynamic curriculum configuration, and live monitoring daemon

## Open Issues, Concerns & Learnings
- **CONCERN** (Autonomous Monitoring & Self-Healing Architecture Specification): Recurring lane-following plateaus require an autonomous monitoring subagent that inspects training logs every 15 minutes, detects crawling traps, corner cliffs, wall-hugging, and policy collapse, and executes self-healing actions (dynamic curriculum shift, automated rollback to landmark checkpoints, watchdog restart).
- **PITFALL** (RealSense 90-deg FOV Cornering Constraint): With RealSense D435i horizontal FOV of 90 degrees (+/-45 deg) and 15-deg downtilt, 90-degree corner roads exit the visible field of view before turn-in. Without lookahead route curvature (Slot 10 lookahead_kappa) and pre-braking, vehicles understeer into the outer wall due to kinetic energy. Pre-braking command must lower speed to 0.35 m/s 1.2m before apex.
- **DECISION** (5-Sector Certified Spawner & Warm-Start Rejection of Rollback): KD-tree geometric validation identified 5 mathematically certified runway sectors across the full track (<8mm error, >=2.5m straightaway runway). Rejected policy rollback because current weights retain high speed (0.7-1.0 m/s), record rewards (+4.7k), and 4.14M steps of visual tuning. Deployed 20% spawner per sector to expose policy to previously unseen track segments.
- **ISSUE** (Issue 103: Shoulder Escape Exploit & Termination Grace Period Dynamics): Unsigned Euclidean distance to boundary markers combined with a 20-step spawn grace period allowed cars drifting off the road to escape termination once beyond the 4cm marker envelope. Solved by introducing signed off-road shoulder departure detection based on lateral error and road width without lowering global lat_err threshold.
- **LEARNING** (Issue 102: Leg 2 Centerline Defect & Rollback Rejection): Audited WP 1871 white line collision (0.1cm clearance) vs physical 40.6cm road width. Swarm proved re-centering to true geometric midpoint preserves all tensor invariants and observation spaces. Rollback strongly rejected as policy weights at Step 3.26M are healthy with 1,001 max length and +4,786 record reward.

## Resume Instructions
1. Reset or clear the screen safely using `/clear` or `/cta-clear`.
2. On fresh context, invoke `/cta-resume` to restore project continuity directly from this file and SQLite databases.
