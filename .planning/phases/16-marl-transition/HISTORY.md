# Phase 16 History: MARL Transition

## Hardened Mastery v2.9 Baseline (2024-05-08)

### Findings: Lateral Precision Drift
Previous versions (v2.8 and below) used a 0.10m safety plateau for lateral error. While this allowed for stable training, it permitted the agent to "wander" within the lane, often hugging the boundaries. This behavior is problematic for MARL where tight formation or overtaking requires high precision.

**Fix**:
Shrinked the plateau to **0.05m** and increased the penalty slope to **10.0**. This makes any deviation beyond 5cm from the centerline significantly more expensive.

```python
def lateral_error_reward(env: ManagerBasedRLEnv) -> torch.Tensor:
    lat_err = env.extras.get("lat_err", torch.zeros(env.num_envs, device=env.device))
    abs_lat = torch.abs(lat_err)
    # Plateau: 1.0 if within 0.05m
    # Penalty: 10.0 per meter outside (Rapid dropoff)
    reward = torch.clamp(1.0 - (torch.clamp(abs_lat - 0.05, min=0.0) * 10.0), min=-1.0)
    return reward
```

### Findings: Steering Jitter
High-frequency steering oscillations were observed, likely caused by the agent attempting to maximize the `heading_alignment_reward` while navigating the discretized physics steps. This jitter reduces traction and causes unrealistic vehicle behavior.

**Fix**:
Increased the `smoothness` reward weight from **5.0** to **15.0** in `ARCProEnvCfg`. This penalizes large deltas between consecutive steering actions more heavily.

```python
# arcpro_env_cfg.py
smoothness = RewTerm(func=mdp_rew.action_rate_smoothness_reward, weight=15.0)
```

### Hyperparameter Tuning: Entropy Coefficient
With the stricter reward signals, an `ent_coef` of 0.005 was found to cause premature convergence to sub-optimal "safe" behaviors.

**Fix**:
Restored `ent_coef` to **0.01** to encourage continued exploration of the narrower high-reward corridor created by the lateral hardening.

### Training Status
- **Initial Metrics**: Survival reached **199 steps** in early Step 0 logs.
- **Convergence**: Stable loss curves and increasing average speed (**0.25 m/s**).

## Gold Master Stabilization (Phase 16 Finalization)

### Findings: Suspension Bounce & False Terminations
The robot exhibited an initial suspension-bounce when spawned, which frequently caused either physics explosions or false-positive base-height terminations right at the beginning of an episode.

**Fix**:
Lowered the spawn height to **12cm** to reduce impact energy, and reduced the base-height termination threshold to **2cm** to allow the robot to settle properly into the track surface.

### Findings: Camera FOV Limitations
The forward-facing camera was not capturing enough of the immediate track surface, reducing the agent's ability to react to immediate curves.

**Fix**:
Adjusted the camera mount to `pos=(0.3, 0.0, 0.35)` and tilted it 30 degrees downward. The new quaternion rotation is `rot=(0.966, 0.0, 0.258, 0.0)`, ensuring the track line is fully visible to the agent's vision system.

```python
# arcpro_robot_cfg.py (Camera Config)
cfg.camera_mount.pos = (0.3, 0.0, 0.35)
cfg.camera_mount.rot = (0.966, 0.0, 0.258, 0.0) # 30 degree tilt down
```

### Findings: Mid-Air Wheel Spin
The agent could dispatch actions while the robot was still dropping into the environment, causing mid-air wheel spin that interfered with traction upon landing.

**Fix**:
Hardened the `WaypointTrackingWrapper` to intercept and force `0.0` actions until the robot's Z height drops below **0.10m**.

```python
# Settle Logic Enforcement
if robot_z_height >= 0.10:
    action = 0.0  # Force zero action until settled
```

### Training Status
- **Gold Master Launch**: The Phase 16 "Gold Master" training run has been successfully launched from Step 0.
- **Initial Metrics**: Actively surviving **~177 steps** right out of the gate.
- **Monitoring**: A watchdog script is actively monitoring the run to detect and warn against any early divergence.
