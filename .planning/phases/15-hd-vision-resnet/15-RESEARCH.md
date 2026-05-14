# Phase 15: HD Vision ResNet-18 Optimization - Research

**Researched:** 2026-05-15
**Domain:** Deep Reinforcement Learning / Vision Backbones / VRAM Optimization
**Confidence:** HIGH

## Summary

This research evaluates the transition from a custom strided CNN to a standard ResNet-18 backbone at 224x224 resolution. At this resolution, lane line visibility (4cm) is maintained at ~1.12 px/cm at a 1m lookahead, providing sufficient signal for precision steering. VRAM costs are manageable (~456MB for 16 environments during rollout), and ImageNet pre-training is recommended to accelerate feature discovery for road markings.

**Primary recommendation:** Use `torchvision.models.resnet18` with pre-trained weights, replacing the `self.cnn` block in `FusionFeaturesExtractor`. Keep the backbone unfrozen (trainable) to allow the model to specialize in road-surface textures and lane-line contrast.

<user_constraints>
## User Constraints (from CONTEXT.md)

### Locked Decisions
- **Scale**: 1.0x Metric (True Physics).
- **Target**: Right Lane Center.
- **Resolution**: 640x360 (Source), 224x224 (ResNet Input).
- **Backbone**: ResNet-18 (Standard).

### the agent's Discretion
- **Pre-training**: Choice of weights (ImageNet vs Scratch).
- **Freezing**: Whether to freeze early layers.
</user_constraints>

<phase_requirements>
## Phase Requirements

| ID | Description | Research Support |
|----|-------------|------------------|
| REQ-RESNET-18 | Implement standard ResNet-18 in policy. | Integration path via `torchvision.models` identified. |
| REQ-VRAM-AUDIT | VRAM check for 12/16 envs. | Audit shows 342MB/456MB activation cost. |
| REQ-HD-VISIBILITY | Verify 4cm line visibility @ 224px. | Calculation confirms 4.48px width @ 1m lookahead. |
</phase_requirements>

## Standard Stack

### Core
| Library | Library ID | Version | Purpose |
|---------|------------|---------|---------|
| torchvision | torchvision | 0.19.1 | Pre-trained ResNet-18 models. |
| torch | torch | 2.4.1 | Neural network backend. |

**Installation:**
```bash
pip install torchvision>=0.19.0 torch>=2.4.0
```

## Architecture Patterns

### ResNet-18 Integration in FusionFeaturesExtractor
The `FusionFeaturesExtractor` should be modified to swap the custom `self.cnn` with a truncated ResNet-18.

```python
# Implementation pattern
from torchvision import models

class FusionFeaturesExtractor(BaseFeaturesExtractor):
    def __init__(self, observation_space, features_dim=256):
        # ...
        backbone = models.resnet18(weights=models.ResNet18_Weights.IMAGENET1K_V1)
        # Truncate at avgpool to get a 512-dim vector
        self.cnn = nn.Sequential(
            nn.AdaptiveAvgPool2d((224, 224)), # Input adapter
            *list(backbone.children())[:-1],
            nn.Flatten()
        )
        # ...
```

## Don't Hand-Roll

| Problem | Don't Build | Use Instead | Why |
|---------|-------------|-------------|-----|
| ResNet Architecture | Custom skip-connections | `torchvision.models.resnet18` | Standardized, pre-optimized, and supports pre-trained weights. |
| Image Normalization | Manual Mean/Std calculation | `transforms.Normalize` | Standard ImageNet constants are calibrated for these weights. |

## Feature Preservation Audit

**Scenario:** 4cm Lane Lines at 1m Lookahead.
- **Camera HFOV:** 90° (Intel RealSense D435i spec).
- **Resolution:** 224x224 (ResNet input).
- **Calculation:**
  - FOV Width @ 1m: $W_{FOV} = 2 \times 1.0 \times \tan(45^\circ) = 2.0$ meters.
  - Pixels-per-cm: $224 \text{ px} / 200 \text{ cm} = 1.12$ px/cm.
  - Line Width in Pixels: $4 \text{ cm} \times 1.12 \text{ px/cm} = 4.48$ pixels.
- **Conclusion:** **CLEARLY VISIBLE.** 4.5 pixels is well above the Nyquist sampling limit for a CNN to detect a high-contrast edge.

## VRAM Audit (ResNet-18 @ 224x224)

| Metric | Per Environment | 12 Envs | 16 Envs |
|--------|-----------------|---------|---------|
| Activations (Inference) | 28.5 MB | 342.3 MB | 456.4 MB |
| Parameters (Weights) | 46.8 MB | 46.8 MB | 46.8 MB |
| **Total (Rollout)** | **75.3 MB** | **389.1 MB** | **503.2 MB** |

*Note: Training VRAM (PPO) with a minibatch of 64 will require approximately 1.8 GB for activations + 200 MB for gradients/optimizer, totaling ~2 GB. This fits comfortably within the 12GB budget.*

## ImageNet Pre-training Assessment

| Aspect | Pre-trained (Unfrozen) | Training from Scratch |
|--------|------------------------|-----------------------|
| **Feature Quality** | **HIGH.** Filters for edges/shapes already exist. | **LOW.** Must learn basic vision from sparse RL rewards. |
| **Convergence** | **FAST.** Focuses on task-logic immediately. | **SLOW.** 1M+ steps spent on "learning to see". |
| **Domain Fit** | Good, adaptable to track texture. | Perfect, but takes significantly longer to train. |
| **Recommendation** | **PRIMARY.** Start with ImageNet weights. | Only if pre-trained fails to generalize. |

## Common Pitfalls

### Pitfall 1: Input Mismatch
**What goes wrong:** The model fails to learn anything despite pre-training.
**Why it happens:** ResNet-18 expects input normalized with ImageNet mean/std (Mean: [0.485, 0.456, 0.406], Std: [0.229, 0.224, 0.225]). If the simulator output is [0, 1] or [0, 255], the features will be "out of distribution".
**How to avoid:** Add a `Normalize` step in the `forward` pass or as a wrapper.

### Pitfall 2: Learning Rate Saturation
**What goes wrong:** Pre-trained weights are destroyed by high RL learning rates.
**Why it happens:** The PPO optimizer uses a high LR (e.g. 3e-4) which might be too aggressive for a pre-trained backbone.
**How to avoid:** Use a differential learning rate or a smaller global LR (1e-4) when using pre-trained weights.

## Sources

### Primary (HIGH confidence)
- `torchvision` Documentation: ResNet implementation details.
- `vram_audit.py`: Empirical measurement of ResNet-18 activations.
- Isaac Lab `ARCProEnvCfg`: Camera and resolution parameters.

## Metadata
**Confidence:** HIGH
**Research date:** 2026-05-15
**Valid until:** 2026-06-15
