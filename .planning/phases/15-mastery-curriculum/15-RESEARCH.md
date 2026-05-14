# Phase 15: Mastery Curriculum - Research

**Researched:** 2026-05-11
**Domain:** RL Reward Engineering & Vision Transfer Learning
**Confidence:** HIGH

## Summary

This research focuses on optimizing the reward structure for the "Mastery Curriculum" phase, where the goal is to achieve full-lap capability with a ResNet-18 vision backbone. We identified that the previous linear lateral error reward was too "loose," leading to oscillations. The new structure introduces a "Comfort Zone" (plateau) and a high-fidelity "Steering Jerk Penalty" to enforce stability. Additionally, we explored leveraging track curvature (`kappa`) to reward predictive steering.

**Primary recommendation:** Transition to a plateau-based lateral reward with a heavy L1 steering jerk penalty to suppress oscillations while maintaining high speed.

## Standard Stack

### Core
| Library | Version | Purpose | Why Standard |
|---------|---------|---------|--------------|
| Isaac Lab | 1.1.0+ | Simulation Framework | Vectorized, high-performance RL |
| PyTorch | 2.1+ | Tensor Math / Rewards | Direct GPU computation for rewards |
| ResNet-18 | torchvision | Vision Backbone | Proven feature extractor with residuals |

## Architecture Patterns

### Comfort-Zone Reward (Plateau)
Instead of a continuous linear penalty that creates constant "micro-corrections," we use a plateau. This allows the robot to ignore tiny errors within a 10cm window but provides a sharp gradient as it approaches the lane boundary.

**Formula:**
$$R_{comfort} = 20.0 \times \text{clamp}\left(1.0 - \frac{|lat\_err| - 0.10}{0.13 - 0.10}, 0.0, 1.0\right)$$

### Steering Jerk Penalty (L1)
To prevent the "jitter" common in vision-based policies, we apply a heavy L1 penalty on the change in steering action. L1 is preferred over L2 here because it penalizes small oscillations more aggressively.

**Formula:**
$$P_{jerk} = -50.0 \times |steer_t - steer_{t-1}|$$

### Curvature-Aware Reward (Holding Angle)
Using the `kappa` ($\kappa$) value from the `TrackManager`, we can reward the agent for steering at the mathematically ideal angle for a given curve. For the F1Tenth model ($L=0.33m$, $\delta_{max}=0.5rad$):

**Ideal Action:** $action\_steer \approx 0.66 \cdot \kappa$

**Formula:**
$$R_{kappa} = 5.0 \times \exp\left(-10.0 \cdot (action\_steer - 0.66 \cdot \kappa)^2\right)$$

## Don't Hand-Roll

| Problem | Don't Build | Use Instead | Why |
|---------|-------------|-------------|-----|
| Weight Transfer | Custom mapping scripts | ImageNet Pre-training | NatureCNN and ResNet-18 weights are structurally incompatible (3 layers vs 18 layers). |
| Smoothness | PID controllers in MDP | Reward Penalties | RL policies learn to optimize the reward; an external PID often fights the policy's learned dynamics. |

## Environment Availability

| Dependency | Required By | Available | Version | Fallback |
|------------|------------|-----------|---------|----------|
| Isaac Lab | Environment | ✓ | 1.1.0 | — |
| TrackManager | Curvature Data | ✓ | — | — |
| ResNet-18 | Vision Policy | ✓ | torchvision | — |

## Common Pitfalls

### Pitfall 1: Over-penalizing Jerk
**What goes wrong:** The robot becomes "scared" to steer, going straight even on curves.
**Why it happens:** Jerk penalty weight is so high it outweighs the lateral error penalty.
**How to avoid:** Ensure the `Comfort-Zone` reward at the center (+20.0) and the `Lateral Error` gradient are strong enough to justify steering.

### Pitfall 2: Static Kappa Mapping
**What goes wrong:** Kappa-based rewards might fail if the robot is not perfectly on the centerline.
**How to avoid:** The Kappa reward should be a secondary bonus, not a primary objective.

## Code Examples

### Optimized Reward Formula (PyTorch)
```python
def comfort_zone_reward(env: ManagerBasedRLEnv) -> torch.Tensor:
    # Source: ARCPro RL Research 2026-05-11
    lat_err = env.extras.get("lat_err", torch.zeros(env.num_envs, device=env.device))
    abs_lat = torch.abs(lat_err)
    
    # 20.0 reward within 10cm, ramp to 0.0 at 13cm
    reward = 20.0 * torch.clamp(1.0 - (abs_lat - 0.10) / (0.13 - 0.10), min=0.0, max=1.0)
    return reward

def steering_smoothness_reward(env: ManagerBasedRLEnv) -> torch.Tensor:
    # Source: ARCPro RL Research 2026-05-11
    current_action = env.action_manager.action
    if "prev_action" not in env.extras:
        return torch.zeros(env.num_envs, device=env.device)
    
    prev_action = env.extras["prev_action"]
    # Heavy L1 penalty on steering change (action index 0)
    penalty = -50.0 * torch.abs(current_action[:, 0] - prev_action[:, 0])
    return penalty
```

## State of the Art: NatureCNN vs ResNet-18

| Feature | NatureCNN | ResNet-18 | Impact |
|--------------|------------------|--------------|--------|
| Depth | 3 Layers | 18 Layers | ResNet captures much more complex hierarchy of features. |
| Weights | Scratch / Atari | ImageNet | ResNet starts with pre-learned "edges" and "shapes." |
| Performance | Fast, shallow | Deep, Residual | ResNet avoids vanishing gradients, allowing deeper visual understanding. |

**'New Eyes, Better Brain' Transition:**
The argument for ResNet-18 over NatureCNN is not just about depth; it's about the **initial state**. NatureCNN starts with random noise and must learn what a "line" is from scratch. ResNet-18 with ImageNet weights already understands line orientations, corners, and color gradients. This allows the RL agent to focus on the **behavioral mapping** (policy) rather than the **perceptual mapping** (vision), significantly accelerating convergence toward a full lap.

## Validation Architecture

### Test Framework
| Property | Value |
|----------|-------|
| Framework | pytest |
| Config file | arcproLab/policy_stack/pytest.ini |
| Quick run command | `pytest arcproLab/policy_stack/tests/test_rewards.py` |
| Full suite command | `pytest arcproLab/policy_stack/test_all_so_far.py` |

### Phase Requirements → Test Map
| Req ID | Behavior | Test Type | Automated Command | File Exists? |
|--------|----------|-----------|-------------------|-------------|
| REW-01 | Comfort-Zone Reward | unit | `pytest arcproLab/policy_stack/tests/test_rewards.py::test_comfort_zone` | ❌ Wave 0 |
| REW-02 | Steering Jerk Penalty | unit | `pytest arcproLab/policy_stack/tests/test_rewards.py::test_jerk_penalty` | ❌ Wave 0 |
| REW-03 | Holding Angle Bonus | unit | `pytest arcproLab/policy_stack/tests/test_rewards.py::test_kappa_reward` | ❌ Wave 0 |

### Wave 0 Gaps
- [ ] `arcproLab/policy_stack/tests/test_rewards.py` — New test file to verify reward math and tensor shapes.

## Sources

### Primary (HIGH confidence)
- `arcproLab/mdp/rewards.py` - Current reward implementation.
- `arcproLab/arcpro_env_cfg.py` - Robot wheelbase and steering constraints.
- `arcproLab/mdp/track_manager.py` - Kappa (curvature) calculation logic.

### Secondary (MEDIUM confidence)
- SB3 Documentation - NatureCNN vs ResNet feature extractors.

## Metadata

**Confidence breakdown:**
- Standard stack: HIGH - Verified in codebase.
- Architecture: HIGH - Math derived from physics/geometry.
- Pitfalls: MEDIUM - Based on common RL training observations.

**Research date:** 2026-05-11
**Valid until:** 2026-06-11
