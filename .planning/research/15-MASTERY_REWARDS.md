# Phase 15: Mastery Rewards & HD Perception - Research

**Researched:** 2026-05-13
**Domain:** RL Optimization / Reward Shaping / CNN Backbones
**Confidence:** HIGH

## Summary

This research deep-dives into memory optimization via `uint8` buffer storage, High-Resolution CNN architectures, and Multi-Boundary reward shaping for centerline mastery. The transition to 640x360 vision requires an 'Aggressive Stride' backbone to maintain VRAM stability, while the reward system is overhauled to provide a sharp 'Magnetic Centerline' and soft boundaries that respect intersection crossing zones.

**Primary recommendation:** Use an 8-4-2 stride CNN backbone for 640x360 input. Implement the exponential magnetic reward with a 20cm hard-reset cutoff for $|lat\_err|$, ensuring gate immunity via `TrackManager` distance checks.

## User Constraints (from CONTEXT.md)

### Locked Decisions
- **Scale**: 1.0x Metric (True Physics).
- **Target**: Right Lane Center (Target path center).
- **Resolution**: 640x360 (HD Vision).

### the agent's Discretion
- **CNN Backbone**: Specific stride/kernel configuration.
- **Reward Weights**: Scaling factor for magnetic reward.

## Research Task 1: High-Res CNN Architecture

### The 100M Parameter Trap
A standard NatureCNN (from Mnih et al. 2015) designed for 84x84 input fails when scaled to 640x360 without significant modification.

| Layer | Type | Kernel/Stride | Output Shape | Parameters |
|-------|------|---------------|--------------|------------|
| Input | RGB | - | 3x360x640 | 0 |
| Conv1 | Conv2d | 8x8, s4 | 32x89x159 | 6,176 |
| Conv2 | Conv2d | 4x4, s2 | 64x43x78 | 32,832 |
| Conv3 | Conv2d | 3x3, s1 | 64x41x76 | 36,928 |
| Flatten | - | - | 199,424 | 0 |
| **FC1** | **Linear** | **-** | **512** | **102,105,600** |

**Total:** ~102.1 Million Parameters.
**Impact:** 408MB VRAM for weights alone. For a 32-env rollout with Adam optimizer, this exceeds 4GB VRAM just for the policy, significantly impacting training speed and limiting concurrent environments.

### Proposed: Aggressive Stride Backbone (8-4-2)
To preserve small features like thin white lines without blurring (which happens with Adaptive Pooling), we use aggressive strides in the early layers.

| Layer | Type | Kernel/Stride | Output Shape | Parameters |
|-------|------|---------------|--------------|------------|
| Conv1 | Conv2d | 8x8, **s8** | 32x45x80 | 6,176 |
| Conv2 | Conv2d | 4x4, **s4** | 64x11x20 | 32,832 |
| Conv3 | Conv2d | 3x3, **s2** | 64x5x9 | 36,928 |
| Flatten | - | - | 2,880 | 0 |
| FC1 | Linear | - | 512 | 1,475,072 |

**Total:** ~1.55 Million Parameters (65x reduction).
**Benefit:** Maintains 640x360 field-of-view (FOV) and fine-feature detection while reducing the flattened vector size to a manageable 2,880 units.

### Training Time Impact
- **Data Throughput**: 160x90 to 640x360 is a 16x increase in pixel data transfer (uint8: 43KB -> 691KB per frame).
- **Computation**: Despite strides, the first conv layer still processes 230,400 pixels.
- **Assessment**: Expect a **3x to 5x slowdown** in steps-per-second (SPS) compared to 160x90. However, the improved perception accuracy for 1.0x metric scale (where 1 pixel @ 160px is ~2cm) justifies the cost.

## Research Task 2: Multi-Boundary Reward Shaping

### Magnetic Centerline (Exponential)
Instead of a linear reward, a squared exponential (Gaussian) kernel provides a high-reward "sweet spot" at the center and drops off rapidly.
- **Function**: $R_{mag} = 30.0 \cdot \exp\left(-\left(\frac{|lat\_err|}{0.05}\right)^2\right)$
- **Characteristics**: 
  - $|lat\_err|=0.00m$: +30.0 (High reward for mastery)
  - $|lat\_err|=0.05m$: +11.0 (Steep drop-off at 5cm)
  - $|lat\_err|=0.10m$: +0.5 (Negligible reward beyond 10cm)

### Soft-Boundary Pressure
Applied to both White and Yellow lines via `TrackManager.compute_marker_distances`.
- **Threshold**: $dist < 0.13m$.
- **Weight**: 10.0 (Linear).
- **Function**: $R_{line} = -10.0 \cdot \frac{0.13 - dist}{0.13}$
- **Immunity**: If $dist\_gate < 0.20m$, set $R_{line} = 0.0$. This prevents the robot from being penalized for crossing stop lines or intersection entry marks (Go-Signal zones).

## Research Task 3: Hard Reset Logic

### The 20cm Cutoff
To enforce strict mastery, the episode terminates if the robot drifts too far from the target.
- **Threshold**: $|lat\_err| > 0.20m$.
- **Rationale**:
  - Centerline: $0.00m$
  - Penalty Zone Starts: $0.17m$ ($dist \approx 0.13m$ from line)
  - Reset Zone: $0.20m$
- **Result**: A 3cm "Life Support" window where the robot feels the soft penalty and must correct before the episode is terminated at 20cm error.

## Research Task 4: ResNet-18 @ 224x224 Optimization

### Feature Preservation Audit
- **Scenario**: 4cm Lane Lines @ 1m lookahead.
- **Resolution**: 224x224 pixels.
- **Result**: **4.48 pixels width**. 
- **Calculated Pixels-per-cm**: 1.12 px/cm.
- **Precision**: Sufficient for "Centerline Mastery" where 1cm errors need to be discernible.

### VRAM Audit (Rollout)
- **12 Environments**: 342.3 MB Activations + 46.8 MB Weights = **389.1 MB**.
- **16 Environments**: 456.4 MB Activations + 46.8 MB Weights = **503.2 MB**.
- **Assessment**: High efficiency for parallel rollout; training (minibatch 64) requires ~2 GB VRAM.

### ImageNet Pre-training Strategy
- **Recommendation**: Use `IMAGENET1K_V1` weights but keep the backbone **unfrozen**.
- **Rationale**: Pre-trained features provide immediate edge/texture detection, but unfrozen weights allow the model to specialize in the specific photometric properties of the Isaac Lab asphalt and marker materials.

## Standard Stack

| Library | Version | Purpose | Why Standard |
|---------|---------|---------|--------------|
| PyTorch | 2.1+ | CNN Backend | Efficient `Stride` implementation. |
| Isaac Lab | 1.1+ | Env Framework | `TrackManager` integration for marker distances. |
| torchvision | 0.19+ | ResNet-18 | Standard vision backbone. |

## Common Pitfalls

### Pitfall: Gate Blindness
**What goes wrong:** Robot refuses to cross intersections.
**Why:** Stop lines are categorized as "white lines," triggering penalties/resets.
**How to avoid:** Use `TrackManager` to identify `dist_gate` and disable line logic within 0.20m of a gate center.

### Pitfall: OOD Pre-training
**What goes wrong:** Pre-trained model performs worse than scratch.
**Why:** Failure to apply ImageNet normalization (Mean/Std).
**How to avoid:** Ensure input is normalized to $[0, 1]$ then transformed via `transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])`.

## Code Examples

### Aggressive Stride Backbone
```python
self.cnn = nn.Sequential(
    nn.Conv2d(3, 32, kernel_size=8, stride=8), # 640x360 -> 80x45
    nn.ReLU(),
    nn.Conv2d(32, 64, kernel_size=4, stride=4), # 80x45 -> 20x11
    nn.ReLU(),
    nn.Conv2d(64, 64, kernel_size=3, stride=2), # 20x11 -> 9x5
    nn.ReLU(),
    nn.Flatten(),
)
```

### ResNet-18 Drop-in Implementation
```python
from torchvision import models

# In FusionFeaturesExtractor.__init__:
backbone = models.resnet18(weights=models.ResNet18_Weights.IMAGENET1K_V1)
self.cnn = nn.Sequential(
    nn.AdaptiveAvgPool2d((224, 224)),
    *list(backbone.children())[:-1], # Truncate before FC
    nn.Flatten()
)
```

### Multi-Boundary Reward logic
```python
def mastery_reward(env):
    dist_y, dist_w, dist_g = tm.compute_marker_distances(pos)
    lat_err = env.extras["lat_err"]
    
    # 1. Magnetic Centerline
    r_mag = 30.0 * torch.exp(-(torch.abs(lat_err) / 0.05)**2)
    
    # 2. Soft Penalties (Immune in Gates)
    in_gate = dist_g < 0.20
    p_white = torch.where((dist_w < 0.13) & (~in_gate), -10.0 * (0.13 - dist_w) / 0.13, 0.0)
    p_yellow = torch.where((dist_y < 0.13) & (~in_gate), -10.0 * (0.13 - dist_y) / 0.13, 0.0)
    
    return r_mag + p_white + p_yellow
```

## Metadata
**Confidence:** HIGH
**Date:** 2026-05-15
