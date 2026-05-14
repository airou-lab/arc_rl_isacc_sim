# Single-Agent Mastery Curriculum - Research

**Researched:** 2026-05-12
**Domain:** RL Curriculum Design / Autonomous Driving / Isaac Sim
**Confidence:** HIGH

## Summary

*See [15-MASTERY_REWARDS.md](15-MASTERY_REWARDS.md) for deep-dive on uint8 buffers and soft boundaries.*

To achieve **Centerline Mastery** in a 1.0x metric scale environment using an RTX 3060, we must balance high-precision lateral control with simulation stability. The research indicates that the current "Killed" crashes at 16 environments are likely due to **System RAM exhaustion** rather than VRAM, as the Adaptive CNN (fixed 128x128 internal resolution) is highly VRAM-efficient.

**Primary recommendation:** Use a **12-environment** parallel configuration with a **3-Phase Boundary Curriculum** that starts with strict constraints to "pin" the agent to the centerline, gradually relaxing them to allow for high-speed recovery and maneuverability.

## User Constraints (from CONTEXT.md)

### Locked Decisions
- **Scale**: 1.0x Metric (True Physics).
- **Target**: Right Lane Center (+2.42m from Yellow Line).
- **Control Rate**: 50Hz (Decimation=10, dt=0.002).
- **Policy**: Vision-only (RGB Camera), Telemetry masked in observation.

### the agent's Discretion
- **Curriculum Schedule**: Define white-line margin transition.
- **Reward Shaping**: Optimize lateral error and line penalty weights.
- **Termination Logic**: Balance soft vs. hard terminations.

## Standard Stack

### Core
| Library | Version | Purpose | Why Standard |
|---------|---------|---------|--------------|
| Adaptive CNN | Custom | Vision Backbone | NatureCNN with `AdaptiveAvgPool2d((128, 128))` for VRAM efficiency. |
| Stable-Baselines3 | 2.3+ | RL Engine | Reliable PPO implementation for continuous control. |
| Isaac Lab | 4.2+ | Simulation | State-of-the-art physics and rendering for robotics. |

### Supporting
| Library | Version | Purpose | When to Use |
|---------|---------|---------|--------------|
| TrackManager | Custom | Geometry Engine | Calculates lateral/heading error and boundary distances. |
| Kornia | 0.7.0+ | Perception Ops | GPU-accelerated HSV filtering and Gaussian Blur. |

## Hybrid Perception Pipeline (Proposed)

To improve sample efficiency and achieve true "Mastery," a transition from raw RGB to structured masks is recommended.

### 1. Mask-CNN Architecture
- **Input**: 640x360 HD.
- **Pre-Processing**: GPU-accelerated HSV filtering (Kornia) to extract Yellow (Centerline) and White (Lane Edges).
- **Observation Space**: 2-channel **Distance-Transformed (DT) Masks**. 
  - Grayscale value represents distance to the nearest line.
  - Achieved via Gaussian Blur on binary masks.
- **Backbone**: CNN starting with **Stride 4** to preserve spatial resolution, followed by a **1x1 Conv Bottleneck** to compress features before the Dense layer.

### 2. Benefits vs. Raw RGB
- **Sample Efficiency**: Eliminates background noise (sky/trees/lighting flares).
- **Gradient Density**: DT Masks provide a continuous signal across the frame, unlike sparse binary masks.
- **VRAM Control**: 1x1 Bottleneck maintains a manageable parameter count (~30M) despite the HD resolution.

## Curriculum Boundaries

The curriculum follows a **Strict-to-Permissive** transition. By starting with a wide "safety margin" (strict reset), we force the agent to find the centerline early. Once the agent demonstrates stability, we shrink the margin to allow the robot to approach the lines for better cornering.

| Phase | Steps | Margin (Hard Reset) | Margin (Soft Penalty) | Allowed Error |
|-------|-------|---------------------|-----------------------|---------------|
| **1: Foundation** | 0 - 500k | 0.25m | 0.30m | $\pm$ 0.05m |
| **2: Discipline** | 500k - 1.5M | 0.18m | 0.25m | $\pm$ 0.12m |
| **3: Mastery** | 1.5M - 3M | 0.13m | 0.18m | $\pm$ 0.17m |

*Note: Margin is the distance to the line from the robot center. Center of lane is ~0.30m from the line. Margin 0.25m means the robot terminates if it drifts more than 0.05m from center.*

## Reward Shaping for Lane-Following

To prioritize the center without inducing "movement fear," the rewards must provide a clear gradient towards the zero-error state while still rewarding forward progress.

### Proposed Weight Map
| Reward Term | Weight | Logic | Intent |
|-------------|--------|-------|--------|
| `speed` | 15.0 | `max(0, speed * 0.5)` | Reward progress but don't let it drown out precision. |
| `lateral_error` | 20.0 | `1.0 - (abs_lat * 10.0)` | Heavy precision incentive. |
| `line_penalty` | 5.0 | `-50.0` if `dist < soft_margin` | "Pain" signal before reset. |
| `smoothness` | 15.0 | `-sq(diff)` | Reduce jitter without freezing steering. |
| `stationary` | 5.0 | `-20.0` if `speed < 0.5` | Prevent the "Safe Stop" strategy. |

**Key insight:** By increasing `lateral_error` weight to 20.0 and lowering `speed` to 15.0, the agent loses more reward from being 10cm off-center than it gains from driving at 1.0 m/s. This mathematically enforces "Mastery."

## VRAM Optimization (RTX 3060)

The "Killed" crashes at 16 environments are a symptom of **resource over-commitment**. 

### Resource Audit (Per 12 Envs)
- **Isaac Sim Base**: ~4.5 GB VRAM
- **Tiled Rendering (12 x 640x360)**: ~2.4 GB VRAM
- **Adaptive CNN Batch (512 x 128x128)**: ~0.1 GB VRAM
- **System RAM (Isaac Sim + SB3 Buffer)**: ~14.0 GB (Estimated)

**Recommendation:** **12 Environments**. 
- Balanced between the "Safe 8" and the "Crash 16".
- Maximize sample efficiency (600 FPS target) while staying under the 12GB VRAM and 16GB/32GB RAM thresholds.
- Disable `enable_cameras` in non-vision training runs to verify baseline stability.

## Termination Logic: Soft vs. Hard

For early training, the agent needs a "buffer zone" where it is penalized but not reset. This prevents the "Reset Loop" where an agent never sees a full curve because it keeps resetting on the entry.

### approach: Dual-Threshold Boundary
1.  **Soft Margin ($d_{soft}$)**:
    - **Condition**: `dist_to_line < d_{soft}`.
    - **Action**: Persistent penalty (`line_penalty`) applied to the reward.
    - **Goal**: Provide a gradient signal that the agent is "getting close to the edge."
2.  **Hard Margin ($d_{hard}$)**:
    - **Condition**: `dist_to_line < d_{hard}`.
    - **Action**: Environment Reset + `termination_penalty` (-500.0).
    - **Goal**: Prevent the agent from learning off-road behaviors.

## Common Pitfalls

### Pitfall 1: "Centerline Phobia"
**What goes wrong:** The agent drives fast but zig-zags wildly across the lane center.
**Why it happens:** `smoothness` penalty is too high relative to `lateral_error`, making any steering correction "expensive."
**How to avoid:** Keep `smoothness` weight $\le$ `lateral_error` weight.

### Pitfall 2: Reset Loops
**What goes wrong:** The agent never completes a lap because the `0.25m` margin is too strict for its initial random behavior.
**Why it happens:** Phase 1 starts before the agent has learned to even move forward.
**How to avoid:** Use a "Discovery Phase" (0 - 100k steps) with no line terminations, only height and stagnation.

## Code Examples

### Hybrid Observation Generation (Conceptual)
```python
def generate_hsv_dt_mask(rgb_tensor):
    hsv = kornia.color.rgb_to_hsv(rgb_tensor)
    yellow = (hsv[:, 0] > 0.1) & (hsv[:, 0] < 0.2) & (hsv[:, 1] > 0.4)
    white = (hsv[:, 1] < 0.2) & (hsv[:, 2] > 0.7)
    masks = torch.stack([yellow.float(), white.float()], dim=1)
    return kornia.filters.gaussian_blur2d(masks, (15, 15), (5.0, 5.0))
```

## Environment Availability

| Dependency | Required By | Available | Version | Fallback |
|------------|------------|-----------|---------|----------|
| Isaac Sim | Rendering | ✓ | 4.2.0 | — |
| RTX 3060 | GPU Accel | ✓ | 12GB | — |
| System RAM | Scene Graph | ✓ | [Check] | Reduce to 8 envs if < 32GB |

## Sources

### Primary (HIGH confidence)
- **Isaac Lab SimulationCfg** - GPU memory management settings.
- **TrackManager.py** - Distance calculation implementation.
- **MDP/Rewards.py** - Current reward structure analysis.

### Secondary (MEDIUM confidence)
- **Phase 15 RESEARCH.md** - VRAM estimates for HD vision.

## Metadata
**Research date:** 2026-05-14
**Valid until:** 2026-06-14
