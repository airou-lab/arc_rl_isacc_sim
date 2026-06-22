# Phase 16 MARL Architecture (RoadGraph) Research

## Current State
- The legacy `RoadGraph` singleton has been moved to `trash/road_graph.py.bak`.
- A new vectorized `RoadManager` exists in `arcproLab/mdp/road_manager.py`. It explicitly replaces the singleton to support independent agent state.
- `RoadManager` maintains `turn_tokens` and `go_signals` as tensors of shape `(num_envs, num_agents)`.
- It initializes gates automatically via `_initialize_gates` by searching the USD stage for `laneGate` primitives and parsing `SignalTurnRelation`.

## Codebase Impact
Despite being fully implemented and vectorized for MARL, **`RoadManager` is currently entirely disconnected from the runtime**.
1. **Telemetry**: In `arcproLab/mdp/observations.py`, `turn_token` (Slot 0) is hardcoded to `0.0`.
2. **Go Signal**: `go_signal` (Slot 1) is handled exclusively by `GoSignalManager`, an independent FSM built around `VisualStopLineDetector`.
3. **Environment**: `RoadManager` is never imported, instantiated, or updated in `arcpro_env_cfg.py` or `arcproLab/mdp/events.py`.

## Refactoring Plan
To fully migrate to MARL in future phases:
1. Wire `RoadManager` into the environment reset/step lifecycle (e.g. `events.py` or environment hooks) so it updates `turn_tokens` per env/agent based on mission logic.
2. In `observations.py` (`get_telemetry_vector`), extract `turn_token` directly from `get_road_manager(num_envs, num_agents).get_nav_commands()`, replacing the hardcoded `0.0`.
3. Reconcile or unify `RoadManager` with `GoSignalManager`. Currently `GoSignalManager` handles stops visually, while `RoadManager` was intended to handle top-down intersection logic. They must operate together so `go_signal` takes the more restrictive bound of either the visual stopline or the intersection right-of-way logic.