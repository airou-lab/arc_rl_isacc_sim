# Session Resume: ARCPro RL Training & Architecture

## Current Status
- **RL Training (Phase 16)**: Actively running via `start_tmux_training.sh` (8 environments).
- **Behavior**: The agent is in the early stages of PPO exploration. It is currently standing still for exactly 1,000 steps (~20 seconds) before hitting the `stagnation` termination condition. It receives ~`-64.5` reward per episode for this. This is expected behavior as it has not yet discovered that driving forward yields `+50.0` reward per step.

## Recent Architectural Changes
1. **Event-Sourced Hybrid Architecture**:
   - `watchdog.py` has been upgraded to a zero-token Python daemon that continuously logs training telemetry to a lightweight SQLite database (`.gsd/archive.db`).
   - If `watchdog.py` detects a critical failure (divergence or extreme stagnation), it uses the Antigravity CLI (`agy run`) to spawn a headless, self-healing agent.
   - The summoned agent is instructed to query the `.gsd/archive.db` using `sqlite3` to view historical context, flexibly modify `arcpro_env_cfg.py` to fix the rewards/penalties, and restart the `tmux` session.

2. **Reward Configuration (`arcpro_env_cfg.py`)**:
   - **Stationary Penalty**: Weight increased to `2.0` (which translates to a significant penalty over the episode). A shape broadcasting bug between `vel` and `go_signal` was fixed using `.squeeze(-1)`.
   - **Progress Reward**: Weight increased massively to `50.0` to heavily incentivize forward momentum once discovered.

3. **Global Skill Created**:
   - `rl-watchdog`: A new global skill added to `~/.gemini/skills/` that teaches agents how to implement this SQLite + Python Daemon self-healing loop in any future codebase.

4. **Background Agents**:
   - A `checker_agent` subagent has been spawned to act as a "Progress Reporter". It wakes up every 30 minutes to check `logs/curriculum_phase1.log` and ping the chat whenever the agent crosses a 500k timestep boundary.

## Next Steps for New Session
1. **Monitor Training**: Check `logs/curriculum_phase1.log` or use `sqlite3 .gsd/archive.db "SELECT * FROM telemetry_events ORDER BY id DESC LIMIT 10;"` to see if the agent has broken out of the 1,000-step stagnation loop and started crashing/learning to steer.
2. **Review Metrics**: Watch for `ep_len_mean` dropping (indicating it is trying to drive and crashing into white lines) and then rising again alongside `speed_mps`.
