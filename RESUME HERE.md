# CLEARSET RESUME HERE

## Active Execution State
- **MILESTONE**: M005: 48-Env Fleet Training & Full-Circuit Lap Mastery
- **Phase**: Phase 3: Autonomous Fleet Supervision & Sector 4 Intersection Convergence
- **Task**: Task 3.2: Intersection Wrong-Lane Remediation & Guidance Curvature Inversion Fix (Issue 120)
- **Next Todo**: Check agent_3000000.pt checkpoint arrival and review 5-sector regression audit verdict.
- **Checkpoint Timestamp**: 2026-11-09T01:32:09.706682+00:00Z
- **Git Commit**: 31f6cb527369dd2baf60a3a38ad10cdcd53611bc

## Persistent State Data Sources
- **Turn Actions Database**: `.cs/cs_turns.db`
- **Codebase RAG Database**: `.cs/cs_codebase.db`
- **Codebase Index Guide**: `cs_codebase_index.yml`
- **Directory Structure**: `cs_directory_structure.yml`

## Recent Completed Actions
- **AUDIT** [WARNING]: Audit of agent_2950000.pt: REGRESSION_DETECTED (2/5 sectors passed)
- **FIX** [SUCCESS]: Self-Healing: PROCESS_HANG: training log is stale (191.9s > 180s)
- **FIX** [SUCCESS]: Self-Healing: PROCESS_HANG: training log is stale (191.6s > 180s)

## Open Issues, Concerns & Learnings
- **LEARNING** (Zero-Trim Straightaway Bias Cured at 2.95M; Corner 1 Exit Understeer at Step 1,353): Under raw policy execution with zero steering trim (--steer_bias_trim 0.0), agent_2950000.pt drove 1,353 steps (1,308 waypoints) from Start Line through the entire West runway and Corner 1 hairpin with zero collisions. Straightaway bias failure at Step 228 is completely resolved. Reset occurred at Corner 1 exit onto Leg 2 runway (X=-14.58m, Y=-7.10m) where centrifugal drift at 0.38 m/s caused the right wheels to touch the outer white line.
- **LEARNING** (GUI Lapping Verification (agent_2500000.pt with -0.14 trim)): Under matched straightaway trim (-0.14 rad), agent_2500000.pt completes a full circuit lap (1,867 steps / 1,381 waypoints / 50.72m) deterministically with zero exploration noise. It cleanly negotiates Corner 1, Leg 2 straightaway, Sector 4 90-degree intersection, and North Straight with less than 6mm lateral error. First reset occurs at Step 1867 exiting Corner 4 due to boundary clipping on the outer white curb as straightaway trim engages.
- **LEARNING** (Cornering Authority vs Straightaway Bias Annealing Mechanism): At 1.0M-1.1M steps, policy has fully mastered tight left turns (Corner 1 hairpin R=0.46m and 90-deg intersection). On straightaways, residual left-turn weight leakage (+0.138 rad) caused yellow line resets on 44cm lanes at 0.60 m/s. Dynamic curriculum immersion on straightaways has dropped bias from +0.1383 (1.0M) to +0.1152 (1.1M) while preserving turn authority (delta steer +0.128 rad).
- **PITFALL** (Policy-head probe index: turn_token is input index 64, not 66): Policy head input = 64 keypoint coords (idx 0-63) + 12 telemetry (idx 64-75). Telemetry slot 0 (turn_token) = idx 64. An earlier probe set idx 66 (slot 2) and reported '+68.8% left-turn authority'; that was wrong, and the 100k/150k delta values in that report were never computed. Always use idx 64 and only report measured values.
- **CONCERN** (Turn token barely drives steering; deterministic policy does not turn at Sector 4): Run 20261022-192651 (21k SpatialSoftmax). GUI debug.sh --sector 4 with agent_200000/best_agent: car centers well on approach, then drives straight through the intersection at ~0.7 m/s and resets at step 140-149 (mean steer -0.25..+0.09; turn needs +0.50..+0.68). Probe of policy head with turn_token=-1.0 (telemetry slot 0 = head input index 64, keypoints zeroed): steer delta +0.0014 (200k), +0.0027 (250k), +0.0060 (300k), +0.0027 (350k); throttle delta ~-0.08 (it brakes but does not steer). Caveat: zero-keypoint probe is synthetic. Training survives turns only via exploration noise (sigma ~0.53). Plan: re-probe each new ckpt; if still ~0 by ~500k, investigate reward/obs coupling of turn_token before training further toward 2M.

## Resume Instructions
1. Reset or clear the screen safely using `/clear` or `/cs-clear`.
2. On fresh context, invoke `/cs-resume` to restore project continuity directly from this file and SQLite databases.
