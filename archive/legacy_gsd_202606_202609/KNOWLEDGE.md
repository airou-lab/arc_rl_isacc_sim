# Project Knowledge

Append-only register of project-specific rules, patterns, and lessons learned.
Agents read this before every unit. Add entries when you discover something worth remembering.
## Rules

| # | Scope | Rule | Why | Added |
|---|-------|------|-----|-------|

## Patterns

| # | Pattern | Where | Notes |
|---|---------|-------|-------|

## Lessons Learned

| # | What Happened | Root Cause | Fix | Scope |
|---|--------------|------------|-----|-------|
| L001 | Agent stuck in crawling local minimum (~0.28 m/s) over 5M steps. | Reward bloat. Massive collision/boundary penalties overpowered the linear speed reward, teaching the agent that crawling is the safest way to maximize survival. | Implement Curriculum Learning (start with zero boundary/jerk penalties and fade them in) and use a Quadratic Target Velocity Reward to force a strict speed optimum. | reinforcement-learning |
| L002 | Agent could not safely navigate at higher speeds (2.0 m/s). | The camera had a 30-degree downward tilt (shortsightedness) and the 'driving blind' termination safety was completely disabled, allowing it to drive off-track without seeing it. | Removed the 30-degree tilt in `TiledCameraCfg` to look straight ahead. Re-implemented `fov_visibility_termination` to terminate if track heading error exceeds half the camera FOV (45 degrees). | computer-vision |
| L003 | Agent failed Curriculum Phase 1, still stuck crawling at 0.23 m/s after 1M steps. | The massive -200 Death Penalty was still active. Crawling for 400 steps before dying yielded a better mathematical score than sprinting for 50 steps before dying. | Set `termination_penalty` weight to `0.0` for Phase 1. The agent must have zero fear of death so it can learn how to sprint first. | reinforcement-learning |
