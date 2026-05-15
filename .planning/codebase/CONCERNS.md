# Codebase Concerns

**Analysis Date:** 2026-05-11

## Tech Debt

**RoadManager Initialization:**
- Issue: `RoadManager` scans the entire USD stage for "laneGate" prims on every startup.
- Files: `arcproLab/mdp/road_manager.py`
- Impact: Slow environment startup times.
- Fix approach: Move gate discovery to the cached `.npz` system used by `TrackManager`.

**Hard-coded Agent Count in Observations:**
- Issue: `mdp/observations.py` hard-codes `num_agents = 1` when accessing `RoadManager`.
- Files: `arcproLab/mdp/observations.py`
- Impact: Blocks multi-agent training where more than one agent exists per environment.
- Fix approach: Fetch `num_agents` dynamically from `env.scene` or environment config.

**Circular Dependency Workarounds:**
- Issue: `WorkerScheduler` and `SchedulerTransport` use local imports within methods to avoid circularity.
- Files: `arcproLab/policy_stack/agent/worker_scheduler.py`
- Impact: Slightly less readable code; potential for runtime errors if not careful.
- Fix approach: Refactor module structure to separate interfaces from implementations.

## Known Bugs

**None reported:**
- MARL transition is in progress; potential for synchronization issues at high `num_envs`.

## Security Considerations

**Unprotected Network Transports:**
- Risk: `IntersectionNodeServer` (Stage 3) may use unprotected network sockets if configured for remote operation.
- Files: `arcproLab/policy_stack/agent/intersection_node_server.py`
- Current mitigation: Default to `LocalTransport` (in-process).

## Performance Bottlenecks

**VRAM Pressure:**
- Problem: Multi-agent environments with HD visual perception (Adaptive CNN) consume significant VRAM.
- Files: `arcproLab/policy_stack/policies/fusion_policy.py`
- Cause: High-resolution images for N agents across B environments.
- Improvement path: Optimize ResNet backbone or implement shared perception features.

## Fragile Areas

**FCFS Arbitration during Jitter:**
- Files: `arcproLab/policy_stack/agent/scheduler_core.py`
- Why fragile: First-Come-First-Served logic depends on monotonic time; network jitter in Stage 3 could cause reordering.
- Safe modification: Use logical timestamps or sequence numbers.

## Scaling Limits

**Arbitration Complexity:**
- Current capacity: Simple O(N^2) path conflict checks.
- Limit: Becomes a bottleneck as the number of agents per intersection increases.
- Scaling path: Spatial partitioning for intersection queries.

## Dependencies at Risk

**sb3-contrib:**
- Risk: `RecurrentPPO` implementation has specific conventions (e.g., bare tuple for LSTM states) that differ from standard SB3.
- Impact: Policy class must maintain messy compatibility shims.

## Missing Critical Features

**Vectorized Scheduler:**
- Problem: `SchedulerCore` is currently a Python-object-based loop.
- Blocks: High-performance training with thousands of environments.
- Priority: High for Phase 17+.

## Test Coverage Gaps

**RoadManager:**
- What's not tested: Vectorized randomization and gate discovery.
- Files: `arcproLab/mdp/road_manager.py`
- Risk: Agents starting with incorrect navigation tokens.
- Priority: Medium.

---

*Concerns audit: 2026-05-11*
