# Phase 11: Retraining & Intersection Logic - Research

**Researched:** 2025-05-24
**Domain:** Reinforcement Learning / Hierarchical Control
**Confidence:** HIGH

## Summary

This phase focuses on integrating the `HierarchicalPathPlanningPolicy` (HPPP) from the `arc_rl_isacc_policy` stack into the 1.0x metric environment of `arcproLab`. Research confirms that HPPP is highly compatible with the current environment, provided that telemetry indices are aligned and action mapping is updated to handle the 3-dimensional [steer, throttle, brake] output. The policy's "kinematic anchors" can be successfully mocked for track-following without a full intersection worker, and the existing dead-reckoning trajectory tracking is reliable at 500Hz for supervised auxiliary losses.

**Primary recommendation:** Use a mocked `turn_token=0.0` for Phase 11 and implement a `CombinedDriveAction` that fuses HPPP's throttle and brake signals before sending velocity commands to the joints.

## Standard Stack

### Core
| Library | Version | Purpose | Why Standard |
|---------|---------|---------|--------------|
| `sb3_contrib` | Latest | Recurrent PPO | Handles the LSTM-based HPPP architecture. |
| `torch` | 2.0+ | Tensor computation | Primary backend for HPPP and Isaac Lab. |
| `numpy` | 1.24+ | Utility | Used for waypoint and trajectory data processing. |

### Supporting
| Library | Version | Purpose | When to Use |
|---------|---------|---------|--------------|
| `WayptTrackingWrapper` | Reference | Trajectory recording | Essential for supervised `waypoint_loss`. |

## Architecture Patterns

### Recommended Project Structure
```
arcproLab/
├── mdp/
│   ├── actions.py         # Add CombinedDriveAction
│   ├── observations.py    # Update to 12-slot HPPP format
│   └── ...
├── policy_stack/
│   ├── policies/
│   │   └── hierarchical_policy.py  # Integrated HPPP
│   └── wrappers/
│       └── waypoint_tracking_wrapper.py # Integrated Wrapper
└── ...
```

### Pattern 1: Mocked Navigation Intent
**What:** Mocking the `turn_token` to `0.0` (STRAIGHT).
**When to use:** During Phase 11 where no intersections exist in the USD scene.
**Example:**
```python
# In observations.py
obs[:, 0] = 0.0  # IDX_TURN_TOKEN
obs[:, 1] = 1.0  # IDX_GO_SIGNAL
```

### Anti-Patterns to Avoid
- **Hardcoding DT in Wrapper:** The `WaypointTrackingWrapper` must dynamically use the environment's `control_dt` (0.05s) rather than a hardcoded 50Hz (0.02s) to ensure trajectory accuracy.
- **Inconsistent Normalization:** Ensure `WAYPOINT_NORM_SCALE` matches the 1.0x scale (set to 2.5 in HPPP).

## Don't Hand-Roll

| Problem | Don't Build | Use Instead | Why |
|---------|-------------|-------------|-----|
| Waypoint Planning | Custom MLP | `HierarchicalPathPlanningPolicy` | HPPP uses kinematic anchors and learned deviations, providing a strong inductive bias. |
| Trajectory Recording | Global list | `TrajectoryStore` (Reference) | Thread-safe singleton avoids data loss during SB3 rollout resets. |
| Dead Reckoning | Custom EKF | `WaypointTrackingWrapper` logic | Simple speed/yaw integration is sufficient for short-term waypoint supervision. |

## Common Pitfalls

### Pitfall 1: Observation Index Mismatch
**What goes wrong:** Policy receives `Relative X/Y` instead of `Turn Token/Go Signal`.
**Why it happens:** `arcproLab` uses a legacy 12-slot format while HPPP uses a navigation-oriented 12-slot format.
**How to avoid:** Explicitly map indices according to HPPP's `IDX_*` constants.

### Pitfall 2: Brake Inversion
**What goes wrong:** `brake` action increases speed or does nothing.
**Why it happens:** `GroupedJointVelocityAction` only expects 1 input.
**How to avoid:** Implement `speed = throttle * (1.0 - brake)` logic in a custom action term.

## Code Examples

### Observation Alignment (mapping to HPPP IDX)
```python
# arcproLab/mdp/observations.py adaptation
obs = torch.zeros((env.num_envs, 12), device=env.device)
obs[:, 0] = 0.0  # TURN_TOKEN (mocked STRAIGHT)
obs[:, 1] = 1.0  # GO_SIGNAL (mocked GO)
obs[:, 2] = 0.0  # GOAL_DIST (mocked 0)
obs[:, 3] = speed
obs[:, 4] = yaw_rate
obs[:, 5] = last_steer
obs[:, 6] = last_throttle
obs[:, 7] = last_brake
obs[:, 8] = 0.0 # lat_err (masked)
obs[:, 9] = 0.0 # head_err (masked)
obs[:, 10] = 0.0 # kappa
obs[:, 11] = total_distance
```

### Combined Drive Action (Brake Mapping)
```python
# Pattern verified from reference IsaacDirectEnv.step()
def process_actions(self, actions: torch.Tensor):
    # actions: (num_envs, 2) -> [throttle, brake]
    throttle = actions[:, 0]
    brake = actions[:, 1]
    
    # Effective speed command
    effective_throttle = throttle * (1.0 - brake)
    
    # Apply to joints
    self._processed_actions[:] = self._offset + self._scale * effective_throttle.unsqueeze(1)
```

## Environment Availability

| Dependency | Required By | Available | Version | Fallback |
|------------|------------|-----------|---------|----------|
| Isaac Lab | Environment | ✓ | 1.1.0 | — |
| SB3-Contrib | Training | ✓ | Latest | — |
| F1Tenth_Metric.usd | Robot Physics | ✓ | 1.0x | — |

## Validation Architecture

### Test Framework
| Property | Value |
|----------|-------|
| Framework | pytest |
| Config file | pytest.ini |
| Quick run command | `pytest tests/test_telemetry.py` |
| Full suite command | `./verify_sim.sh` |

### Phase Requirements → Test Map
| Req ID | Behavior | Test Type | Automated Command | File Exists? |
|--------|----------|-----------|-------------------|-------------|
| PH11-01 | Telemetry matches HPPP `IDX_*` | Unit | `pytest tests/test_telemetry.py` | ❌ Wave 0 |
| PH11-02 | Brake reduces wheel velocity | Integration | `python arcproLab/scripts/verify_brake.py` | ❌ Wave 0 |
| PH11-03 | Waypoint loss decreases | Training | `python arcproLab/scripts/train_policy.py --check-aux-loss` | ❌ Wave 0 |

## Sources

### Primary (HIGH confidence)
- `../arc_rl_isacc_policy/policies/hierarchical_policy.py` - Source of HPPP architecture and indices.
- `../arc_rl_isacc_policy/isaac_direct_env.py` - Verified brake mapping logic.
- `arcproLab/mdp/observations.py` - Current telemetry layout for comparison.

### Secondary (MEDIUM confidence)
- `../arc_rl_isacc_policy/wrappers/waypoint_tracking_wrapper.py` - Trajectory tracking logic.

## Metadata

**Confidence breakdown:**
- Standard stack: HIGH - Directly using reference policy stack.
- Architecture: HIGH - Observation/Action mapping is straightforward.
- Pitfalls: HIGH - Identified clear mismatch in telemetry indices.

**Research date:** 2025-05-24
**Valid until:** 2025-06-24
