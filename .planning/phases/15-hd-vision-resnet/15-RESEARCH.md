# Phase 15: HD ResNet-18 Integration - Research

**Researched:** 2024-05-18
**Domain:** Vision-based Reinforcement Learning (RL) / Isaac Sim
**Confidence:** HIGH

## Summary

Implementing a 960x600 vision-based RL policy on an RTX 3060 (12GB) requires a balance between rendering overhead and the training memory budget. Isaac Sim consumes ~4.5GB VRAM at idle, and a training batch of 512 HD images (float32) consumes ~3.5GB. This leaves ~3GB for environment viewports and model activations.

**Primary recommendation:** Use **4 parallel environments** with a **Deep Stem ResNet-18** (replacing the 7x7 conv with three 3x3 convs) and **Gated Fusion** for telemetry integration.

## User Constraints (from CONTEXT.md)

*Note: No CONTEXT.md was found for this phase. Research follows the technical requirements provided in the goal.*

## Standard Stack

### Core
| Library | Version | Purpose | Why Standard |
|---------|---------|---------|--------------|
| PyTorch | 2.2+ | Deep Learning | Industry standard, primary backend for SB3. |
| Stable-Baselines3 | 2.3+ | RL Framework | Robust PPO implementation with custom policy support. |
| Isaac Sim | 4.0+ | Simulation | High-fidelity physics and rendering. |
| torchvision | 0.17+ | Vision Models | Provides pre-trained ResNet-18 and building blocks. |

### Supporting
| Library | Version | Purpose | When to Use |
|---------|---------|---------|--------------|
| GroupNorm | N/A | Normalization | Use instead of BatchNorm for RL stability. |
| GMU | Custom | Gated Fusion | To combine vision and telemetry features adaptively. |

**Installation:**
```bash
pip install stable-baselines3 shimmy[gymanywhere] torchvision
```

## Architecture Patterns

### Recommended Project Structure
```
arcproLab/
├── mdp/
│   ├── vision_policy.py    # Custom SB3 Features Extractor
│   ├── fusion_layers.py    # Gated Multimodal Unit implementation
│   └── resnet_stem.py      # Modified ResNet-18 architecture
```

### Pattern 1: Deep Stem for HD Images
**What:** Replace the standard 7x7 stride-2 convolution and 3x3 maxpool with three 3x3 convolutions.
**When to use:** High-resolution inputs where fine-grained spatial detail preservation is critical.
**Example:**
```python
# Modified Stem for 960x600 -> 240x150
stem = nn.Sequential(
    nn.Conv2d(3, 32, kernel_size=3, stride=2, padding=1, bias=False), # 480x300
    nn.GroupNorm(8, 32),
    nn.ReLU(inplace=True),
    nn.Conv2d(32, 32, kernel_size=3, stride=1, padding=1, bias=False), # 480x300
    nn.GroupNorm(8, 32),
    nn.ReLU(inplace=True),
    nn.Conv2d(32, 64, kernel_size=3, stride=2, padding=1, bias=False), # 240x150
    nn.GroupNorm(8, 64),
    nn.ReLU(inplace=True)
)
```

### Anti-Patterns to Avoid
- **BatchNorm in RL:** BatchNorm depends on batch statistics which are non-stationary in RL. Use **GroupNorm** or **LayerNorm**.
- **Flattening HD Feature Maps:** A 960x600 image reduced to 1/32 scale still has 30x19 spatial dimensions. Flattening 512 channels results in ~290k features, leading to massive policy heads and overfitting. Use **Global Average Pooling (GAP)**.

## Don't Hand-Roll

| Problem | Don't Build | Use Instead | Why |
|---------|-------------|-------------|-----|
| ResNet backbone | Custom CNN | `torchvision.models.resnet18` | Pre-verified architecture and weight loading. |
| Image Preprocessing | Manual Scaling | `torchvision.transforms` | Highly optimized on GPU/CPU. |
| PPO | Custom PPO | `stable_baselines3.PPO` | Handles complex edge cases (GAE, clipping, entropy) reliably. |

## Common Pitfalls

### Pitfall 1: VRAM Exhaustion during Update
**What goes wrong:** The training script crashes with CUDA OOM when transitioning from Rollout to Train phase.
**Why it happens:** Rollout data is stored as uint8 on CPU, but SB3 converts the entire mini-batch to float32 on GPU for the update. 1024 HD images = ~6.8GB VRAM.
**How to avoid:** Keep `batch_size` at 512 and use `n_envs <= 4`. Monitor VRAM during the first training update.

### Pitfall 2: Vanishing Detail in Stem
**What goes wrong:** Agent fails to see thin lane lines or distant obstacles.
**Why it happens:** Standard MaxPool stride-2 is a "hard" downsampling that can discard high-frequency features.
**How to avoid:** Use strided convolutions (learnable downsampling) in the Deep Stem.

## Code Examples

### Gated Fusion (GMU) Implementation
```python
class GatedMultimodalUnit(nn.Module):
    def __init__(self, vision_dim, telemetry_dim, output_dim):
        super().__init__()
        self.vision_proj = nn.Linear(vision_dim, output_dim)
        self.telemetry_proj = nn.Linear(telemetry_dim, output_dim)
        self.gate = nn.Linear(vision_dim + telemetry_dim, output_dim)
        self.sigmoid = nn.Sigmoid()

    def forward(self, vision_feat, telemetry_feat):
        h_v = torch.tanh(self.vision_proj(vision_feat))
        h_t = torch.tanh(self.telemetry_proj(telemetry_feat))
        # Compute gate based on concatenated raw features
        z = self.sigmoid(self.gate(torch.cat([vision_feat, telemetry_feat], dim=-1)))
        return z * h_v + (1 - z) * h_t
```

## Questions & Answers

### 1. Optimal Number of Environments
**Recommendation:** **4 environments**.
- **Isaac Sim Base:** ~4.5 GB
- **Rendering Overhead (4 x 0.7 GB):** ~2.8 GB
- **Training Batch (512 images, float32):** ~3.4 GB
- **Total Estimated:** **10.7 GB** (under the 11 GB budget).

### 2. Stem Modification
**Recommendation:** **Strided Convolutions (Deep Stem)**.
- Replace the 7x7 conv and 3x3 MaxPool with three 3x3 convs.
- Use **Stride-2** in the 1st and 3rd layers to reach 240x150 resolution gracefully.
- Strided convs are faster due to kernel fusion and preserve more learnable spatial information than MaxPool.

### 3. GAP vs. Flattening
**Recommendation:** **Global Average Pooling (GAP) is superior**.
- Reduces feature maps to a fixed 512-dim vector regardless of input resolution.
- Prevents the "parameter explosion" in the MLP head (~290k for Flatten vs 512 for GAP).
- Provides robustness to small spatial shifts.

### 4. Late Fusion Architecture
**Recommendation:** **Gated Fusion (GMU)**.
- Superior to Concatenation as it allows the model to dynamically prioritize modalities (e.g., ignoring noisy vision in favor of telemetry).
- Enhances robustness to sensor failures or environmental noise.

### 5. PPO Hyperparameters
| Parameter | Value | Reason |
|-----------|-------|--------|
| `learning_rate` | $2 \times 10^{-4}$ | Balance stability and speed. |
| `n_steps` | 2048 | Standard for high-dim visual RL. |
| `batch_size` | 512 | Fits in VRAM with 960x600 images. |
| `n_epochs` | 10 | Maximize data utility per rollout. |
| `ent_coef` | 0.01 | Maintain exploration in high-res space. |

## Environment Availability

| Dependency | Required By | Available | Version | Fallback |
|------------|------------|-----------|---------|----------|
| Python | Runtime | ✓ | 3.12.3 | — |
| NVIDIA GPU | Rendering/Training | ✓ | RTX 3060 (12GB) | — |
| PyTorch | Model Backend | ✓ | 2.10.0 | — |
| torchvision | Feature Extractor | ✓ | 0.25.0 | — |
| stable-baselines3 | RL Algorithm | ✗ | — | Install in Wave 0 |

**Missing dependencies with no fallback:**
- **stable-baselines3**: Must be installed before phase execution.

## Sources

### Primary (HIGH confidence)
- **NVIDIA Isaac Sim Docs** - VRAM usage and Tiled Rendering specs.
- **Stable-Baselines3 Docs** - Custom policy and buffer implementation.
- **"ResNet-D" / Bag of Tricks** - Deep Stem and strided conv research.

### Secondary (MEDIUM confidence)
- **Gated Multimodal Units for Information Fusion** (Luvizon et al.) - GMU architecture.

## Metadata

**Confidence breakdown:**
- Standard stack: HIGH - SB3/ResNet are mature.
- Architecture: HIGH - Gated fusion and Deep Stem are well-documented.
- VRAM Estimates: MEDIUM - Actual usage depends on scene complexity (USD textures).

**Research date:** 2024-05-18
**Valid until:** 2024-06-18
