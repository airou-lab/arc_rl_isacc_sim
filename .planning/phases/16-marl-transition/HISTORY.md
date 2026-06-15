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
