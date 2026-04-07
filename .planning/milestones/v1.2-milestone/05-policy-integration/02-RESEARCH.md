# Phase 5-02: Hierarchical Policy & Telemetry - Research

**Researched:** 2026-03-27
**Domain:** Autonomous Driving RL / Hierarchical Planning / Isaac Lab
**Confidence:** HIGH

## Summary

This phase focuses on the integration of the Hierarchical Policy architecture, the 12-float telemetry protocol, and Gaussian-weighted rewards in the 1.0x metric environment. The project has already transitioned to "True Physics" mode, and we are now aligning the RL stack with the physical reality of the 1.0x metric scale.

**Primary recommendation:** Use the `HierarchicalPathPlanningPolicy` from `policy_stack/` with the confirmed 12-float mapping and implement 'Hybrid Racer' Gaussian rewards to achieve smoother lane-following behavior.

## User Constraints (from CONTEXT.md)

*(Note: No CONTEXT.md found, using project instructions and ROADMAP.md/STATE.md context)*

### Locked Decisions
- **Framework:** Isaac Lab (Manager-Based RL).
- **Metric Scale:** 1.0x (True Physics).
- **Architecture:** Hierarchical (Planning Head + Control Head).
- **Observation Space:** Dual-stream (160x90 RGB Image + 12-Float Telemetry).

### the agent's Discretion
- **Reward Math:** Implementation details of "Hybrid Racer" Gaussian rewards.
- **Spawning Logic:** Specific randomization parameters for lane-aligned resets.
- **Protocol Details:** Exact mapping of telemetry indices [0-11].

## Phase Requirements

| ID | Description | Research Support |
|----|-------------|------------------|
| REQ-TELEMETRY-12 | Implement 12-float protocol in `observations.py`. | Confirmed mapping from `hierarchical_policy.py`. |
| REQ-REWARD-HYBRID | Implement Gaussian rewards in `rewards.py`. | Confirmed math from `docs/RL_REWARDS.md`. |
| REQ-SPAWN-LANE | Implement lane-aligned resets in `events.py`. | `TrackManager` waypoint sampling logic identified. |
| REQ-POLICY-INTEGRATE | Integrate policy stack in `arcproLab/policy_stack/`. | Existing file structure verified and needs minor cleanup. |

## Standard Stack

### Core
| Library | Version | Purpose | Why Standard |
|---------|---------|---------|--------------|
| Isaac Lab | 1.1.0+ | Simulation Environment | NVIDIA's official framework for robot learning. |
| Stable Baselines3 | 2.1.0+ | RL Algorithms | Industry standard for reliable RL baselines. |
| SB3-Contrib | 2.1.0+ | Recurrent PPO | Provides RecurrentPPO for hierarchical policies. |
| PyTorch | 2.2.0+ | DL Backend | Native support in Isaac Lab and SB3. |

### Supporting
| Library | Version | Purpose | When to Use |
|---------|---------|---------|--------------|
| NumPy | < 2.0.0 | Math/Waypoints | Data manipulation for TrackManager. |
| Torchvision | Current | CNN Layers | Pre-trained ResNet/CNN backbones for perception. |

**Installation:**
```bash
pip install sb3-contrib gymnasium==0.29.1
```

## Architecture Patterns

### Recommended Project Structure
```
arcproLab/
├── mdp/
│   ├── observations.py   # Implement get_telemetry_vector (12-float)
│   ├── rewards.py        # Implement hybrid_racer_reward
│   ├── events.py         # Implement reset_robot_to_lane
│   └── track_manager.py  # Add get_closest_waypoint_curvature
└── policy_stack/
    ├── __init__.py
    ├── fusion_policy.py  # CNN + Physics Extractor
    └── hierarchical_policy.py # Main Hierarchical Policy
```

### Pattern 1: 12-Float Telemetry Protocol
**What:** Mapping Isaac Lab sensor data to a fixed 12-element vector for the Hierarchical Policy.
**Confirmed Mapping:**
- `0`: `IDX_TURN_TOKEN` (Discrete turn command {-1, 0, 1})
- `1`: `IDX_GO_SIGNAL` (Go/wait {0.0, 1.0})
- `2`: `IDX_GOAL_DIST` (Goal distance, usually 0)
- `3`: `IDX_SPEED` (Forward speed m/s)
- `4`: `IDX_YAW_RATE` (Yaw rate rad/s)
- `5`: `IDX_LAST_STEER` (Previous steer)
- `6`: `IDX_LAST_THR` (Previous throttle)
- `7`: `IDX_LAST_BRK` (Previous brake)
- `8`: `IDX_LAT_ERR` (Lateral error m)
- `9`: `IDX_HDG_ERR` (Heading error rad)
- `10`: `IDX_KAPPA` (Path curvature 1/m)
- `11`: `IDX_DS` (Distance traveled m)

### Pattern 2: Hybrid Racer (Gaussian Reward)
**What:** Reward function that "pulls" the robot to the center using a bell curve.
**Example Math:**
```python
# Source: docs/RL_REWARDS.md
lane_reward = 2.0 * torch.exp(-(lat_err**2) / (2 * 0.25**2))
speed_reward = speed * 2.0
total_reward = lane_reward + speed_reward - collision_penalty
```

## Don't Hand-Roll

| Problem | Don't Build | Use Instead | Why |
|---------|-------------|-------------|-----|
| Recurrent PPO | Custom LSTM wrapper | `sb3_contrib.RecurrentPPO` | Handles sequence buffering and state management correctly. |
| Feature Fusion | Custom Concat layer | `FusionFeaturesExtractor` | Already includes necessary LayerNorm for heterogeneous streams. |
| Point Projection | Manual distance math | `TrackManager.compute_errors` | Vectorized and handles track orientation properly. |

## Environment Availability

| Dependency | Required By | Available | Version | Fallback |
|------------|------------|-----------|---------|----------|
| Isaac Lab | Environment | ✓ | 1.1.0 | — |
| SB3 | Training | ✓ | 2.1.0 | — |
| SB3-Contrib | Recurrent Policy | ✗ | — | `pip install sb3-contrib` |
| USD Track | Scene | ✓ | — | `no_graph_sim_final.usd` |

**Missing dependencies with no fallback:**
- `sb3-contrib`: Required for `HierarchicalPathPlanningPolicy`. Must be installed in Wave 0.

## Common Pitfalls

### Pitfall 1: Telemetry Scale Mismatch
**What goes wrong:** Training fails to converge because `lateral_error` is in centimeters while `speed` is in meters.
**Why it happens:** Legacy ARCPro used diverse units; 1.0x metric mode requires strict SI units.
**How to avoid:** Normalize all inputs or rely on `FusionFeaturesExtractor`'s LayerNorm.

### Pitfall 2: Reset Pose Jitter
**What goes wrong:** Robot resets into the ground or unstable state.
**Why it happens:** Spawning exactly at Z=0 when track geometry might have slight variations.
**How to avoid:** Spawn at Z=0.08m (matching wheel radius + safety margin) and let physics settle.

## Code Examples

### 12-Float Telemetry Implementation
```python
# mdp/observations.py
def get_telemetry_vector(env, asset_cfg=SceneEntityCfg("robot")):
    obs = torch.zeros((env.num_envs, 12), device=env.device)
    asset = env.scene[asset_cfg.name]
    
    # Speed (m/s)
    obs[:, 3] = asset.data.root_lin_vel_b[:, 0]
    # Yaw Rate (rad/s)
    obs[:, 4] = asset.data.root_ang_vel_b[:, 2]
    # Lateral & Heading Error
    lat_err, head_err = tm.compute_errors(asset.data.root_pos_w, yaw)
    obs[:, 8] = lat_err
    obs[:, 9] = head_err
    # Distance
    obs[:, 11] = env.extras["distance"]
    return obs
```

### Lane-Aligned Spawning
```python
# mdp/events.py
def reset_robot_to_lane(env, env_ids):
    tm = get_track_manager()
    # Pick random waypoint
    indices = torch.randint(0, len(tm.waypoints), (len(env_ids),))
    wp = tm.waypoints[indices]
    
    pos = wp[:, :2] # (x, y)
    yaw = wp[:, 2]  # yaw
    
    # Add jitter
    pos += (torch.rand((len(env_ids), 2)) - 0.5) * 0.1 # 10cm jitter
    
    # Teleport
    asset.write_root_pose_to_sim(...)
```

### Confirmed Strategy (Final Discussion)
- **Drive Mode (FWD)**: Confirmed robot is Front-Wheel Drive. Only `Joint_Drive_FL` and `Joint_Drive_FR` will be used for acceleration. Steering is also front-only.
- **Physical Obstacles**: The "Invisible Barriers" issue is considered **FIXED** by the transition to `no_graph_sim_final.usd`. Hard collision penalties (e.g., -50.0) can now be applied safely.
- **Endless Mode / Worker Bypass**: The Hierarchical Worker module will be bypassed or set to a "Go Straight" constant for the lane-following phase. Index 2 (Goal Distance) is set to `0.0` for endless navigation.

## Open Questions (Resolved)

1. **`IDX_KAPPA` Computation:** Does `TrackManager` need a pre-computed curvature column in `track_centerline.npy`?
   - *Resolution:* Update `track_manager.py` to compute curvature (1/m) from waypoints during initialization using a 3-point circle-fit.
2. **`IDX_DS` Persistence:** Should distance traveled persist across resets for some metrics?
   - *Resolution:* Reset `distance` in `env.extras` upon every episode reset.

## Sources

### Primary (HIGH confidence)
- `arcproLab/policy_stack/hierarchical_policy.py` - Confirmed 12-float mapping.
- `docs/RL_REWARDS.md` - Confirmed Gaussian reward math.
- `arcproLab/mdp/track_manager.py` - Confirmed error computation logic.

### Secondary (MEDIUM confidence)
- `arcproLab/mdp/observations.py` - Current partial implementation of telemetry.

## Metadata

**Confidence breakdown:**
- Standard stack: HIGH - Core libraries are well-defined.
- Architecture: HIGH - Hierarchical policy code is already present.
- Pitfalls: MEDIUM - Scaling issues are common in metric migrations.

**Research date:** 2026-03-27
**Valid until:** 2026-04-26
