# CTA RESUME HERE

## Active Execution State
- **MILESTONE**: M001: ARCPro Visual RL Full-Lap Navigation
- **Phase**: Phase 1: End-to-End Visual Driving on Closed Circuit
- **Task**: Task 1.2: 64.5m City Circuit Corner Transition & Loop Closure
- **Next Todo**: Continue PPO training toward 5.0M steps on the clean 64.5m parametric circuit. Monitor episode length crossing >1,000 steps as the agent masters the South intersection corner.
- **Checkpoint Timestamp**: 2026-08-26T22:50:08.094410+00:00Z
- **Git Commit**: 19e0898d67dd5f560131ba718ee2ace1e22b8168

## Persistent State Data Sources
- **Turn Actions Database**: `.cta/cta_turns.db`
- **Codebase RAG Database**: `.cta/cta_codebase.db`
- **Codebase Index Guide**: `cta_codebase_index.yml`
- **Directory Structure**: `cta_directory_structure.yml`

## Recent Completed Actions
- **FIX** [SUCCESS]: Resolved silent jerk_penalty no-op bug by wiring prev_action tracking in mdp/rewards.py and mdp/events.py. Centered steering offset (0.0) and scaled lateral_error to 10.0 for stable, smooth driving.
- **FIX** [SUCCESS]: Replaced corrupted 150k nearest-neighbor track_centerline.npy with clean 64.5m continuous parametric closed circuit. Resumed training from peak best_agent.pt (+2,247 reward).
- **REFACTOR** [SUCCESS]: Reverted uncommitted changes back to proven 1.2M baseline with heading alignment and waypoint progress. Resumed training in tmux from agent_1217700.pt.

## Open Issues, Concerns & Learnings
- **LEARNING** [steering]: jerk_penalty requires active prev_action cache in env.extras to penalize bang-bang oscillation.

## Resume Instructions
1. Reset or clear the screen safely using `/clear` or `/cta-clear`.
2. On fresh context, invoke `/cta-resume` to restore project continuity directly from this file and SQLite databases.
