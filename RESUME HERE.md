# CLEARSET RESUME HERE

## Active Execution State
- **MILESTONE**: M005: 48-Env Fleet Training & Full-Circuit Lap Mastery
- **Phase**: Phase 3: Autonomous Fleet Training Supervision & Sector 4 Convergence
- **Task**: Task 3.1: Supervise 48-camera fleet training (crossing 10M+ transitions) toward full lap milestone
- **Next Todo**: Monitor 48-env training in tmux session 'training'; execute dual headless audit (Oracle & 5-sector) at Step 200k.
- **Checkpoint Timestamp**: 2026-10-18T17:50:00.000000+00:00Z
- **Git Commit**: 686561e0d658b26eeff1891886407bc48375f77f
- **Roadmap Specification**: [`.planning/ROADMAP.md`](.planning/ROADMAP.md)

## Strategic Milestone Pipeline
1. **M005 (CURRENT)**: 48-Env High-Density Fleet Training & Full-Circuit Lap Mastery (>1,840 steps / 50.72m circuit).
2. **M006 (NEXT)**: Smart Intersection Signals & Dynamic Traffic Light Coordination (`go_signal_manager.py` FSM, `intersection_node_server.py`, stop-bar dwell compliance).
3. **M007 (UPCOMING)**: Multi-Robot Fleet Interaction & Forward Obstacle Detection / Avoidance (RealSense 32-bin depth corridor slicing, TTC headway penalty, opponent traffic, radar Doppler / 120 FPS MIPI abstraction).

## Persistent State Data Sources
- **Turn Actions Database**: `.cs/cs_turns.db` (or `.cta/cta_turns.db`)
- **Codebase RAG Database**: `.cs/cs_codebase.db` (or `.cta/cta_codebase.db`)
- **Codebase Index Guide**: `cs_codebase_index.yml`
- **Directory Structure**: `cs_directory_structure.yml`
- **Full Project Roadmap**: `.planning/ROADMAP.md`

## Recent Completed Actions
- **AUDIT** [COMPLETED]: Dual headless audit of `agent_150000.pt` vs Golden Oracle Stanley controller and 5-sector regression suite.
  - Sector 2 (Corner 1 hairpin): PASS (200/200, 0.63 m/s)
  - Sector 3 (Leg 2 sprint): PASS (200/200, 0.90 m/s)
  - Sector 5 (North straight & bend): PASS (200/200, 0.63 m/s)
  - Sector 4 (Intersection turn-in): FAIL (Crashed @ Step 120, overspeeding at 0.78 m/s without left steering lock)
- **PLANNING** [COMPLETED]: Formulated M005 $\to$ M006 $\to$ M007 ClearSet engineering roadmap and obstacle avoidance architecture.
- **EXECUTION** [SUCCESS]: Resumed 48-env fleet training under Autonomous Sentinel with clean VRAM footprint (1.5 GB $\to$ 7.2 GB).

## Open Issues, Concerns & Learnings
- **LEARNING** (Intersection Turn-In Failure Root Cause & Golden Oracle Comparison): Side-by-side benchmarking proved the policy has mastered C1, Leg 2, and the North Bend, but treats the Sector 4 intersection entry as an ongoing straightaway sprint (0.78 - 0.88 m/s) while holding neutral/slight right steering (-0.04 to -0.12). Oracle initiates left lock (+0.21) at X=-5.8m. Active `intersection_expert_guidance` (+100.0) and -400.0 collision penalty are actively pulling policy toward pre-braking and early turn lock.
- **LEARNING** (Sensor-Agnostic Obstacle Avoidance Architecture): Established 32-bin virtual range scan from horizontal RealSense depth corridor slicing (rows 90-130 min-pooling). Replaces heavy, high-latency monocular ViTs (DepthAnything) with <0.2ms zero-GPU compute, scaling seamlessly to automotive mmWave radar (Doppler velocity) and 120 FPS MIPI CSI-2 global shutter cameras.
- **LEARNING** (Corner 1 & Corner 4 Centerline Waypoint Defect Resolution): Re-centered arcs using radius 0.50m centered at (-15.705, -6.60) and (-15.705, 6.60), ensuring minimum clearance to both white and yellow lines >= 17.7cm everywhere on the circuit.

## Resume Instructions
1. Reset or clear the screen safely using `/clear` or `/cs-clear`.
2. On fresh context, invoke `/cs-resume` to restore project continuity directly from this file and SQLite databases.
