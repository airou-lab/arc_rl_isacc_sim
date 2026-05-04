# Phase 9: Training Stabilization - Research (Physics Refinement)

**Researched:** 2026-04-14
**Domain:** Isaac Lab Physics, Articulation Stability, Mass Scaling
**Confidence:** HIGH

## Summary

This research focuses on the implementation of **REQ-SIM-FIDELITY**: Reducing the robot mass from 30kg to a realistic 5kg while maintaining simulation stability and physical consistency. The core challenge in low-mass configurations is preventing numerical "jitter" and "explosions" caused by high actuator gains and large mass ratios between links.

**Primary recommendation:** Apply a linear scaling law ($1/6 \times$) to all motor parameters (effort limits, stiffness, damping) to preserve dynamics, and use the `armature` parameter (0.01) in joint configurations to provide the numerical inertia required for solver stability.

## Standard Stack

### Core
| Library | Version | Purpose | Why Standard |
|---------|---------|---------|--------------|
| Isaac Lab | 4.2.0+ | Simulation Framework | Primary framework for the project. |
| PhysX 5.x | Latest | Physics Engine | TGS solver provides the necessary stability for articulations. |

### Supporting
| Library | Version | Purpose | When to Use |
|---------|---------|---------|--------------|
| pxr (USD) | Latest | Scene Manipulation | Used via `mdp/spawner.py` to apply mass and CoM overrides. |

## Architecture Patterns

### Scaling Laws for Mass Reduction
When reducing mass by a factor $\alpha$ (where $\alpha = M_{new}/M_{old}$), the following properties must be scaled linearly to maintain identical acceleration profiles and control bandwidth:

- **Effort Limit ($T_{max}$):** $T_{new} = \alpha T_{old}$
- **Joint Stiffness ($K$):** $K_{new} = \alpha K_{old}$
- **Joint Damping ($C$):** $C_{new} = \alpha C_{old}$

For this phase ($\alpha = 5/30 = 1/6$):
| Parameter | Old (30kg) | New (5kg) | Calculation |
|-----------|------------|-----------|-------------|
| Steering Stiffness | 100.0 | 16.67 | $100 \times 1/6$ |
| Steering Damping | 5.0 | 0.83 | $5 \times 1/6$ |
| Steering Effort | 50.0 | 8.33 | $50 \times 1/6$ |
| Throttle Damping | 5.0 | 0.83 | $5 \times 1/6$ |
| Throttle Effort | 20.0 | 3.33 | $20 \times 1/6$ |

### Stability Best Practices (Low Mass)
1.  **TGS Solver:** Always use `solver_type=1` (Temporal Gauss-Seidel). It is significantly more stable for high-stiffness joints and low mass ratios.
2.  **Solver Iterations:** Increase `solver_position_iteration_count` on the Articulation Root to 16 or 32 (up from 8).
3.  **Joint Armature:** Add a small `armature` value (e.g., 0.01) to all `ImplicitActuatorCfg` entries. This adds a virtual inertia to the joint diagonal, which prevents the solver from oscillating when the physical link mass is very small.
4.  **Time Step:** Maintain `dt=0.002` (500Hz). Small scale physics (1.0x metric) requires high temporal resolution to resolve contact and joint constraints.

## Don't Hand-Roll

| Problem | Don't Build | Use Instead | Why |
|---------|-------------|-------------|-----|
| Motor Inertia | Custom damping loops | `armature` in ActuatorCfg | Built into PhysX; mathematically more stable than software-level damping. |
| Friction Compensation | Complex grip logic | Torque Scaling | Scaling torque naturally prevents wheel spin by matching the reduced normal force. |

## Common Pitfalls

### Pitfall 1: Over-Powered Motors
**What goes wrong:** If mass is reduced but torque limits stay at 30kg levels, the RL policy will experience extreme wheel spin and jitter, as the motor is effectively "infinitely" strong relative to the car's weight.
**How to avoid:** Scale `effort_limit_sim` strictly by the mass ratio.

### Pitfall 2: Numerical Bounce
**What goes wrong:** Light robots (5kg) can "vibrate" against the road surface if `max_depenetration_velocity` or `bounce_threshold_velocity` are too high.
**How to avoid:** Keep `bounce_threshold_velocity=0.5` and consider adding a small amount of `linear_damping` (0.01) to the chassis rigid body properties.

## Code Examples

### 1. Updated `arcpro_robot_cfg.py`
```python
# Predicted changes for 5kg configuration
def spawn_f1tenth_preset(prim_path, cfg, translation=None, orientation=None):
    mass_overrides = {
        "Chassis": 5.0,     # Reduced from 30.0
        "Wheel_.*": 0.2,    # Reduced from 1.0 (Realistic for 1/10)
        "Knuckle_.*": 0.05, # Reduced from 0.1
    }
    return arcpro_spawner.spawn_f1tenth(prim_path, cfg, translation, orientation, mass_overrides=mass_overrides)

# ... inside ArcProRobotCfg ...
    actuators: dict = {
        "steering": ImplicitActuatorCfg(
            joint_names_expr=["Joint_Steer_.*"],
            effort_limit_sim=8.33, # Scaled 1/6
            stiffness=16.67,       # Scaled 1/6
            damping=0.83,          # Scaled 1/6
            armature=0.01,         # Stability helper
        ),
        "throttle": ImplicitActuatorCfg(
            joint_names_expr=["Joint_Drive_.*"], 
            effort_limit_sim=3.33, # Scaled 1/6
            stiffness=0.0,
            damping=0.83,          # Scaled 1/6
            armature=0.01,         # Stability helper
        ),
    }
```

## Tire Friction Investigation

**Finding:** The downward force ($F_g = mg$) decreases linearly with mass. Since friction limit $F_s = \mu F_g$, the torque required to induce slip also decreases linearly: $T_{slip} = \mu mg R_{wheel}$.

**Adjustment Needed:**
- No adjustment to the friction coefficient ($\mu$) is strictly necessary if the motor torque is scaled.
- If excessive spin persists, the project's high-friction material ($\mu=2.0$) from `trash/tools/apply_friction.py` can be applied to the wheels.

## Environment Availability

| Dependency | Required By | Available | Version | Fallback |
|------------|------------|-----------|---------|----------|
| Isaac Sim | Simulation | ✓ | 4.2.0 | — |
| PhysX GPU | Physics | ✓ | 5.x | — |

## Validation Architecture

### Test Framework
| Property | Value |
|----------|-------|
| Framework | pytest |
| Command | `pytest tests/test_physics_scaling.py` |

### Phase Requirements → Test Map
| Req ID | Behavior | Test Type | Automated Command |
|--------|----------|-----------|-------------------|
| PHYS-01 | Robot mass is 5kg | unit | `python3 arcproLab/scripts/check_mass_distribution.py` |
| PHYS-02 | Steering settles in < 0.1s | integration | `python3 arcproLab/scripts/verify_drive.py` |
| PHYS-03 | No wheel spin at 50% throttle | integration | `python3 arcproLab/scripts/verify_drive.py` |

## Sources

### Primary (HIGH confidence)
- **PhysX Articulation Documentation** - Stability guides for TGS and Armature.
- **Isaac Lab Actuator Documentation** - `ImplicitActuatorCfg` parameters.
- **Project Archives** - `.planning/phases/10-asset-downscaling/10-RESEARCH.md`

## Metadata
**Confidence breakdown:**
- Scaling laws: HIGH (Linear physics)
- Stability: HIGH (Tested patterns in Isaac Lab)
- Friction: MEDIUM (Depends on specific USD material properties)

**Research date:** 2026-04-14
**Valid until:** 2026-05-14
