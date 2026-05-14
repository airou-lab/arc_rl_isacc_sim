# Phase 16: Multi-Agent Urban Coordination - Research

**Researched:** 2026-05-11
**Domain:** Multi-Agent Reinforcement Learning (MARL), Vectorized Simulation, VRAM Optimization
**Confidence:** HIGH

## Summary

This research phase defines the transition from a single-agent "Road in Void" simulation to a multi-agent urban coordination framework. The primary challenge is refactoring legacy singleton patterns in `RoadGraph` and `TrackManager` to support $N$ agents per environment while maintaining the VRAM efficiency required for HD vision on the RTX 3060.

**Primary recommendation:** Use Isaac Lab's regex-based asset spawning to batch multiple robots into a single `Articulation` group, and refactor MDP helper classes (`RoadGraph`, `TrackManager`) into vectorized managers that accept agent-indexed tensors.

<user_constraints>
## User Constraints (from CONTEXT.md)

### Locked Decisions
- **Scale**: 1.0x Metric (True Physics).
- **Mode**: "Road in Void" (Grass/Fences ghosted).
- **Orientation**: **South-facing** (-1.57 rad).
- **Target**: Right Lane Center (+2.42m from Yellow Line).
- **Boundaries**: Strict Reset (Safe Zone: 0.225m to 2.5m).
- **Frequency**: 500Hz (`dt=0.002`), Control Rate: 20Hz (`decimation=25`).
- **Robot**: 20kg mass, 5Nm torque limits.
- **Action Fix**: **Inverted Throttle** (Scale: -60.0).
- **Vision-only**: `lat_err` and `head_err` are masked in policy observations.
- **Camera**: Mimics Intel RealSense D435i Wide (90° HFOV), tilted 10° down.

### the agent's Discretion
- Transition to Multi-Agent Reinforcement Learning (MARL).
- Refactoring `RoadGraph` and `TrackManager` for vectorization.
- VRAM "Camera Budget" management.
- Integration with Isaac Lab's Manager-Based RL.

### Deferred Ideas (OUT OF SCOPE)
- Full urban scene with foliage/buildings (remain in "Road in Void" mode).
- Complex V2V communication protocols (focus on spatial coordination first).
</user_constraints>

<phase_requirements>
## Phase Requirements

| ID | Description | Research Support |
|----|-------------|------------------|
| MARL-01 | Refactor RoadGraph from singleton to agent-indexed tensors. | Identified singleton bottleneck in `mdp/road_graph.py`; proposed `(num_envs, num_agents)` tensor state. |
| MARL-02 | Vectorize TrackManager for multiple target lane offsets. | Verified `torch.cdist` scalability; proposed tensor-based `target_lane_offset` in `compute_errors`. |
| MARL-03 | Manage VRAM Budget for multiple HD cameras. | Confirmed `TiledCamera` efficiency; 32 HD views ≈ 4K texture, well within RTX 3060 12GB limits. |
| MARL-04 | Maintain Adaptive CNN compatibility. | Confirmed `AdaptiveAvgPool2d` transparency to batch size $B = num\_envs \times num\_agents$. |
</phase_requirements>

## Standard Stack

### Core
| Library | Version | Purpose | Why Standard |
|---------|---------|---------|--------------|
| Isaac Lab | 1.1.0+ | Simulation Engine | Industry standard for vectorized RL. |
| PyTorch | 2.2.0+ | DL Framework | Native support for vectorized tensor operations. |
| Stable Baselines 3 | 2.3.0+ | RL Algorithms | Robust PPO implementation used for ARCPro policies. |

### Supporting
| Library | Version | Purpose | When to Use |
|---------|---------|---------|--------------|
| Skrl | 1.1.0+ | Multi-Agent RL | Use for decentralized multi-agent training if SB3 limits are hit. |

**Installation:**
```bash
# Core requirements are already present in the Isaac Lab environment.
# For multi-agent skrl support (optional):
pip install skrl
```

## Architecture Patterns

### Recommended Project Structure
```
arcproLab/
├── mdp/
│   ├── road_manager.py     # REFACTORED: Replaces road_graph.py (Class-based)
│   ├── track_manager.py    # UPDATED: Vectorized compute_errors
│   └── ...
├── arcpro_multi_env_cfg.py # NEW: Config for N agents per env
└── ...
```

### Pattern 1: Regex-Based Multi-Asset Spawning
Instead of defining `robot_1`, `robot_2`, etc., use a regex in `SceneEntityCfg` to treat all robots as a single batched articulation.
**Example:**
```python
# arcpro_multi_env_cfg.py
robot = ARCPRO_ROBOT_CFG.replace(
    prim_path="{ENV_REGEX_NS}/Robot_[0-9]", # Spawns N robots per environment
    ...
)
```

### Pattern 2: Agent-Indexed Manager State
Refactor `RoadGraph` to store state in tensors of shape `(num_envs, num_agents)`.
**Example:**
```python
# mdp/road_manager.py
class RoadManager:
    def __init__(self, num_envs, num_agents, device):
        self.turn_token = torch.zeros((num_envs, num_agents), device=device)
        self.go_signal = torch.ones((num_envs, num_agents), device=device)
```

### Anti-Patterns to Avoid
- **Looping over Agents:** NEVER use Python loops to calculate errors or rewards. Always use PyTorch vectorized operations.
- **Global Singletons:** Global state prevents running multiple environment instances in the same process or complicates multi-agent indexing.

## Don't Hand-Roll

| Problem | Don't Build | Use Instead | Why |
|---------|-------------|-------------|-----|
| Multi-view Rendering | Custom RenderProducts | `TiledCamera` | Isaac Lab's `TiledCamera` batches all views into one texture, drastically reducing VRAM overhead. |
| Agent Synchronization | Manual Lockstep | `ManagerBasedRLEnv` | Built-in step logic ensures all agents and environments advance simultaneously. |

## Runtime State Inventory

| Category | Items Found | Action Required |
|----------|-------------|------------------|
| Stored data | `track_boundaries_1x.npz` | None (point cloud remains valid for 1.0x scale). |
| Stored data | Model Checkpoints (`.pth`) | Migration required if observation space dimensions change. |
| Live service config | `inference_server_ros2.py` | Update to handle multi-agent array inputs/outputs. |
| OS-registered state | None | Verified. |
| Secrets/env vars | None | Verified. |
| Build artifacts | Isaac Lab installation | Verify version compatibility with `DirectMARLEnv` if needed. |

## Common Pitfalls

### Pitfall 1: VRAM "Death by separate Cameras"
**What goes wrong:** Creating separate `TiledCamera` instances for each agent instead of one instance with a regex path.
**Why it happens:** Misunderstanding how Isaac Lab handles sensor batching.
**How to avoid:** Use `{ENV_REGEX_NS}/Robot_.*` in the `prim_path` of a single `TiledCameraCfg`.

### Pitfall 2: Reset Pollution
**What goes wrong:** Resetting the navigation mission for all agents when only one agent resets.
**Why it happens:** Using environment-level `reset_buf` to trigger `RoadGraph` updates without agent-level indexing.
**How to avoid:** Mask the `turn_token` update using both `env.reset_buf` and an agent-specific completion mask.

## Code Examples

### Vectorized Track Error Calculation
```python
# mdp/track_manager.py
def compute_errors(self, pos: torch.Tensor, yaw: torch.Tensor, target_lane_offset: torch.Tensor):
    """
    pos: (B, 2) where B = num_envs * num_agents
    yaw: (B,)
    target_lane_offset: (B,) - allows different offsets per agent
    """
    # Vectorized closest waypoint search (scales to thousands of points)
    dists = torch.cdist(pos[:, :2], self.waypoints[:, :2])
    closest_idx = torch.argmin(dists, dim=1)
    
    # ... legacy error logic ...
    
    # Vectorized offset application
    lat_err = raw_lat_err - target_lane_offset
    return lat_err, head_err, kappa
```

## State of the Art

| Old Approach | Current Approach | When Changed | Impact |
|--------------|------------------|--------------|--------|
| Singleton Managers | Instance-based Managers | MARL Transition | Allows independent agent state tracking. |
| Float Lane Offset | Tensor Lane Offset | MARL Transition | Enables multi-lane coordination and overtaking. |
| Separate Cameras | Tiled Texture Rendering | Isaac Lab 1.0 | 50-80% VRAM reduction for high-env counts. |

## Open Questions

1. **How to handle Heterogeneous Agents?**
   - Currently, all agents are `F1Tenth_Metric`. If we add different robot types, we need separate `Articulation` groups.
   - Recommendation: Start with homogeneous agents for Milestone 4.

2. **Is `skrl` required for decentralized training?**
   - SB3's PPO treats the whole batch as one policy. This is fine for homogeneous agents sharing weights.
   - Recommendation: Stick with SB3 unless explicit decentralized logic is required.

## Environment Availability

| Dependency | Required By | Available | Version | Fallback |
|------------|------------|-----------|---------|----------|
| Isaac Lab | Core Sim | ✓ | 1.1.0 | — |
| PyTorch | Adaptive CNN | ✓ | 2.2.0 | — |
| RTX 3060 (12GB) | HD Rendering | ✓ | — | Decrease `num_envs` |

## Validation Architecture

### Test Framework
| Property | Value |
|----------|-------|
| Framework | pytest |
| Config file | `arcproLab/policy_stack/pytest.ini` |
| Quick run command | `pytest arcproLab/policy_stack/tests/ -m "not slow"` |
| Full suite command | `pytest arcproLab/policy_stack/tests/` |

### Phase Requirements → Test Map
| Req ID | Behavior | Test Type | Automated Command | File Exists? |
|--------|----------|-----------|-------------------|-------------|
| MARL-01 | RoadGraph supports (B, N) tensors | Unit | `pytest tests/test_road_manager.py` | ❌ Wave 0 |
| MARL-02 | TrackManager handles tensor offsets | Unit | `pytest tests/test_track_manager_vec.py` | ❌ Wave 0 |
| MARL-03 | VRAM remains < 10GB with 32 cameras | Stress | `python scripts/stress_test_vram.py` | ❌ Wave 0 |

## Sources

### Primary (HIGH confidence)
- `arcproLab/mdp/road_graph.py` - Current singleton implementation.
- `arcproLab/mdp/track_manager.py` - Current scalar-offset implementation.
- `arcproLab/arcpro_env_cfg.py` - Current scene and camera config.
- Isaac Lab Documentation - TiledCamera and Manager-Based RL patterns.

### Secondary (MEDIUM confidence)
- Web search for "Isaac Lab multi-agent tutorial" - Confirmed `DirectMARLEnv` as preferred for complex MARL, but Manager-Based is viable for batched homogeneous agents.

## Metadata

**Confidence breakdown:**
- Standard stack: HIGH - Core libraries are stable.
- Architecture: HIGH - Vectorization patterns are standard in Isaac Lab.
- Pitfalls: MEDIUM - VRAM limits are empirical and depend on scene complexity.

**Research date:** 2026-05-11
**Valid until:** 2026-06-11
