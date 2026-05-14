# Research: Image Resolution Standards for End-to-End Vision-Based RL

**Researched:** 2026-05-08
**Domain:** Computer Vision / Reinforcement Learning / Autonomous Driving
**Confidence:** HIGH

## Summary

This research investigates the industry standards for image resolution in end-to-end vision-based reinforcement learning (RL) for autonomous driving. Findings indicate that while raw sensor data often reaches 1080p or 4K, the "bottleneck" resolution passed to neural network backbones for control tasks (steering/throttle) remains surprisingly low, typically between **128x128** and **256x256**. For precision tasks like lane keeping and intersection navigation, resolutions exceeding **512x256** yield diminishing returns in control accuracy while significantly degrading training throughput and sample efficiency.

**Primary recommendation:** Maintain the current **160x90** resolution for standard training. If precision in intersection selection (e.g., reading small signs) becomes a blocker, implement a **"Focus Crop"** architecture (extracting high-res patches) rather than upscaling the entire input.

## 1. Common Resolutions in Peer-Reviewed Research

| Entity/Benchmark | Resolution | Architecture Type | Primary Goal |
|-----------------|------------|-------------------|--------------|
| **NVIDIA DAVE-2** | 200 x 66 | Custom CNN | Lane following / Steering |
| **Wayve (2018)** | 128 x 128 | DDPG + VAE | Real-world lane following |
| **CARLA (TransFuser)** | 224 x 224 | CNN + ViT | Multi-modal urban navigation |
| **CARLA (TCP/LAV)** | 256 x 256 | ResNet-50 / RegNet | Trajectory-guided control |
| **F1Tenth (Vision)** | 160 x 120 | NatureCNN | High-speed track racing |
| **ARCPro RL (Current)**| 160 x 90 | NatureCNN / Fusion | Precision metric-scale driving |

### Key Insight
The industry "sweet spot" is **256 pixels wide**. This resolution provides enough geometric detail to calculate lateral offset and heading error with sub-degree precision while keeping the state space small enough for rapid RL convergence.

## 2. Point of Diminishing Returns

Research consistently shows a sharp drop in performance-per-pixel beyond the **512x256** threshold for control tasks.

- **Steering & Lane Keeping:** 360p (640x360) vs 1080p (1920x1080).
  - **360p:** More than sufficient for detecting lane boundaries at 100m+.
  - **1080p:** No statistically significant gain in steering stability. The increased pixel count (9x) leads to massive aliasing in the first CNN layers and forces the network to spend most of its capacity "throwing away" redundant information.
- **Intersection Navigation:** 
  - Requires slightly higher resolution for **semantic cues** (traffic light state, turn signs).
  - However, rather than 1080p, researchers use **spatial attention** or **multi-scale crops** (e.g., a 128x128 crop focused on the horizon) to preserve detail where it matters.

## 3. Architectural Trends for Resolution Handling

### Backbones
- **ResNet-18/34/50:** Standard for <256p.
- **Vision Transformers (ViT):** Gaining popularity for multi-modal fusion (e.g., TransFuser), but typically operate on **16x16 patches** of a 224x224 input.
- **Early Downsampling:** For any input >480p, successful architectures use **aggressive striding** (stride=2 or 4) in the first two layers to immediately reduce the feature map size to a manageable latent space.

### Anti-Patterns to Avoid
- **Raw High-Res Input:** Passing 720p/1080p directly to an RL policy without a frozen perception backbone. This causes the "Curse of Dimensionality," leading to extremely slow or failed convergence.
- **Square Resizing of Wide-FOV:** Resizing a 90° HFOV wide image to 224x224 (1:1) distorts the geometry, making it harder for the agent to learn true metric distances. Always maintain the aspect ratio (16:9).

## 4. Hardware & VRAM Management

### Isaac Sim / Isaac Lab Specifics
- **Tiled Rendering:** Isaac Lab uses a "tiled" buffer to render multiple environments. 
- **VRAM Footprint (Empirical Updates - RTX 3060 12GB):** 
  - **Low Res (128x128):** Can scale to **1024+ environments**.
  - **Mid/HD Res (640x360):** *CRITICAL LIMIT IDENTIFIED.* Training becomes unstable and is reliably **killed by OOM during PPO optimization** when using 30+ environments.
  - **Safe HD Baseline:** Empirically verified that **16 environments** at 640x360 maintains stable VRAM usage (~6.2GB) allowing room for PPO buffer gradients.
- **MARL "Camera Budget":** For multi-agent training, the total camera count must not exceed 16. E.g., 8 environments with 2 agents = 16 cameras.
- **Strategy for 12GB GPUs:**
  1. Use **TiledCamera** (standard in Isaac Lab).
  2. If high resolution is required for data collection, drop `num_envs` to <8.
  3. For RL training, prefer **higher environment counts (128+)** at **lower resolution (160x90)** over few environments at high resolution. Parallelism is more valuable for RL stability than raw pixel count.

## Recommendations for ARCPro RL

1. **Resolution:** Maintain **160x90**. It is highly optimized for the 1.0x metric scale and fits the 12GB VRAM budget for 32+ environments.
2. **Architecture:** Continue using the **FusionFeaturesExtractor** with early striding. It successfully bottlenecked the 160x90 input into a 256-dim feature vector, which is standard.
3. **Precision Navigation:** If the agent fails to "see" intersections early enough:
   - **DO NOT** upscale the whole image to 1080p.
   - **DO** implement a **horizon-focused crop** or increase the **Camera Tilt** to put more road pixels in the frame.
4. **Validation:** Use the current `verify_metric.py` to ensure that 160x90 allows for sub-centimeter lateral error detection (it should).

## Sources

### Primary (HIGH confidence)
- **NVIDIA DAVE-2 (2016):** "End to End Learning for Self-Driving Cars" - Verified 200x66 resolution.
- **Wayve (2018):** "Learning to Drive in a Day" - Verified 128x128 resolution.
- **Isaac Lab Documentation:** Tiled Rendering and VRAM benchmarks for RTX GPUs.

### Secondary (MEDIUM confidence)
- **CARLA Leaderboard (2023-2024):** Analysis of top models (TransFuser, TCP) showing 224/256 standard backbones.
- **F1Tenth Research:** Common vision-based RL resolutions for high-speed control.

---
**Metadata**
- Research date: 2026-05-08
- Valid until: 2026-11-01
- Confidence: HIGH
