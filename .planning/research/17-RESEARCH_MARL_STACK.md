# Phase 17: MARL Stack & V2V Logic - Research

**Researched:** 2026-05-11
**Domain:** Multi-Agent Reinforcement Learning (MARL), Isaac Lab, V2V Coordination
**Confidence:** HIGH

## Summary

This research evaluates the transition from Stable Baselines 3 (SB3) to a native MARL framework within Isaac Lab, targeting Milestone 4 (MARL, Intersection Coordination). Given the strict 12GB VRAM constraint (RTX 3060) and the requirement for multi-agent HD vision, **SKRL** emerges as the optimal choice. It provides native support for multi-agent GPU tensors and avoids the severe CPU-GPU transfer bottlenecks inherent in SB3.

**Primary recommendation:** Create a separate policy branch for SKRL (`policy_stack_skrl/`) rather than refactoring the current SB3 HPPO Bridge. This isolates the high-risk framework migration and allows native usage of SKRL's MAPPO/IPPO algorithms without breaking the existing stable single-agent baseline.

<user_constraints>
## User Constraints

### Locked Decisions
- Target: Milestone 4 (MARL, Intersection Coordination).
- Constraint: RTX 3060 (12GB VRAM).
- Current Stack: Stable Baselines 3 (SB3), Custom HPPO Bridge, 160x90 and 640x360 vision.

### the agent's Discretion
- MARL Framework choice (SKRL vs RL Games vs SB3).
- Approach to refactoring `FusionFeaturesExtractor` for 5D tensors.
- V2V Communication vector design (1D).
- "Go Signal Precedence" and "Random Turn" logic in `RoadGraph`.
</user_constraints>

## Standard Stack

### Core MARL Framework Comparison
| Framework | Version | Isaac Lab Support | MARL Capability | Recommendation for ARCPro |
|-----------|---------|-------------------|-----------------|---------------------------|
| **SKRL** | 1.3+ | **Native / Excellent** | Native MAPPO/IPPO | **Recommended.** Best balance of modularity, readability, and native GPU tensor support. |
| **RL Games** | 1.6+ | Native / Excellent | Native MAPPO | **Not recommended.** Highly performant but config-heavy and very difficult to customize for custom V2V/HD Vision fusion. |
| **SB3** | 2.2+ | Bridge required | No native MARL | **Deprecate for MARL.** Severe CPU-GPU bottlenecks; does not handle `(B, N, ...)` tensors cleanly. |

### Supporting Tools
| Library | Purpose | When to Use |
|---------|---------|-------------|
| PyTorch `torch.nn.AdaptiveAvgPool2d` | VRAM mitigation | Essential for HD vision (640x360) downscaling before Conv layers to fit within 12GB VRAM. |

**Installation:**
```bash
pip install skrl
```

## Architecture Patterns

### Pattern 1: SKRL 5D Tensor Vision Extractor
**What:** SKRL handles the multi-agent vision tensor `(num_envs, num_agents, C, H, W)` by requiring a custom `Model` class that reshapes the input to 4D for the CNN and back to 3D for the policy head.
**When to use:** Replacing the SB3 `FusionFeaturesExtractor`.
**Example:**
```python
# SKRL requires inheriting from their Model base and a Distribution Mixin
class FusionMARLModel(GaussianMixin, Model):
    def compute(self, inputs, role):
        img = inputs["observations"]["image"] # Shape: (B, N, C, H, W)
        vec = inputs["observations"]["vec"]   # Shape: (B, N, V)
        
        B, N, C, H, W = img.shape
        
        # 1. Flatten Envs and Agents for CNN -> (B*N, C, H, W)
        img_flat = img.view(B * N, C, H, W)
        
        # 2. Extract features
        cnn_feats = self.cnn(img_flat) # Output: (B*N, F)
        
        # 3. Concatenate with vector telemetry
        vec_flat = vec.view(B * N, -1)
        fused = torch.cat([cnn_feats, vec_flat], dim=1)
        
        # 4. MLP Output
        action_flat = self.mlp(fused)
        
        # 5. Restore Multi-Agent shape -> (B, N, ActionDim)
        return action_flat.view(B, N, -1), {"log_std": self.log_std_parameter}, {}
```

### Pattern 2: V2V Communication Observation
**What:** A 1D tensor injected into the `vec` observation space representing the nearest interacting agent.
**Vector Design (6 dims):**
`[rel_x, rel_y, rel_yaw, rel_vel_x, rel_vel_y, go_signal_status]`
- Coordinates must be transformed into the **ego-agent's local frame**.
- `go_signal_status`: `1.0` (Ego has right-of-way), `-1.0` (Ego must yield), `0.0` (No interaction/Neutral).

### Pattern 3: Sim-Governed Go Signal Precedence
**What:** The `RoadManager` acts as a central traffic arbiter.
**Logic:** First-In-First-Out (FIFO) queue based on approach zone entry.
1. When Agent A enters the "intersection approach zone" (distance to center < threshold), `RoadManager` registers A.
2. If the intersection is clear, A gets `go_signal = 1.0`.
3. If Agent B enters while A is crossing, B is queued and gets `go_signal = -1.0`.
4. When A passes the exit waypoint, A's token is cleared, and B is dequeued (`go_signal = 1.0`).

### Pattern 4: Persistent Random Turn Intent
**What:** Agents must commit to a turn direction (-1, 0, 1) and hold it.
**Implementation:** `RoadManager` tracks a tensor `self.turn_intents = torch.zeros((num_envs, num_agents))`.
- Updated **only** on environment reset or when the agent successfully clears an intersection.
- Prevents policy flickering where the agent oscillates between deciding to turn left or right mid-intersection.

## Don't Hand-Roll

| Problem | Don't Build | Use Instead | Why |
|---------|-------------|-------------|-----|
| MARL PPO Loop | Custom Multi-Agent wrapper for SB3 | SKRL's `MAPPO` | Handling advantages, values, and shared critic architectures for (B, N) tensors is error-prone. SKRL handles this natively. |
| VRAM Management | Complex gradient checkpointing | `AdaptiveAvgPool2d` + SKRL `uint8` rollout buffers | At 12GB VRAM, the rollout buffer dominates. Storing 640x360 frames as `float32` will instantly OOM. Downscale early. |
| V2V Raycasting | PhysX Raycasts to find agents | Tensor distance matrix | CPU/PhysX raycasts are slow. Use PyTorch `torch.cdist` on agent position tensors to find the nearest neighbor. |

## Common Pitfalls

### Pitfall 1: SB3 CPU-GPU Data Transfer Bottleneck
**What goes wrong:** Training SPS (Steps Per Second) drops to < 100.
**Why it happens:** SB3 natively expects NumPy arrays or unbatched Dict spaces. The Isaac Lab wrapper forces GPU tensors to CPU for SB3, then back to GPU.
**How to avoid:** Abandon SB3 for MARL. Use SKRL, which keeps all transitions on the GPU.

### Pitfall 2: VRAM Explosion with Multi-Agent Vision
**What goes wrong:** CUDA Out of Memory during the first PPO update.
**Why it happens:** `B * N * C * H * W` during the PPO backward pass is massive. 32 envs * 2 agents * 3 channels * 640 * 360 * float32 = huge activation memory.
**How to avoid:**
1. Insert `nn.AdaptiveAvgPool2d((128, 128))` as the *very first* layer in the CNN to immediately crush the spatial dimensions before any convolutions.
2. Ensure SKRL is configured to store observations in `uint8` if possible.

## Code Examples

### Vectorized V2V Nearest Neighbor Logic
```python
# Inside arcproLab.mdp.observations (Vectorized)
def get_v2v_observations(env, ego_positions, ego_quats, ego_velocities):
    B, N, _ = ego_positions.shape
    
    # Simple 2-agent scenario for Milestone 4
    # Agent 0 looks at Agent 1, Agent 1 looks at Agent 0
    other_idx = torch.tensor([1, 0], device=ego_positions.device)
    
    other_pos = ego_positions[:, other_idx, :]
    other_vel = ego_velocities[:, other_idx, :]
    
    # Calculate relative positions in world frame
    rel_pos_world = other_pos - ego_positions
    
    # Rotate into ego local frame (pseudo-code, requires quaternion math utility)
    rel_pos_local = math_utils.quat_rotate_inverse(ego_quats, rel_pos_world)
    rel_vel_local = math_utils.quat_rotate_inverse(ego_quats, other_vel - ego_velocities)
    
    # Extract Go Signal from RoadManager
    go_signal = env.road_manager.get_go_signals() # Shape: (B, N, 1)
    
    # Assemble [rel_x, rel_y, rel_yaw, rel_vel_x, rel_vel_y, go_signal]
    # ...
```

## State of the Art

| Old Approach | Current Approach | When Changed | Impact |
|--------------|------------------|--------------|--------|
| SB3 HPPO Bridge | SKRL MAPPO | Phase 17 (Planned) | Allows native (B, N) tensor processing, massive SPS boost, eliminates CPU transfer overhead. |
| Global RoadGraph Singleton | Vectorized RoadManager | Phase 16 | Enables per-agent intersection queuing and turn intents for MARL. |

## Open Questions

1. **SKRL Shared vs Independent Vision Encoder**
   - What we know: 12GB VRAM is strict.
   - What's unclear: Can we share the CNN weights across all agents in SKRL while keeping independent MLPs?
   - Recommendation: Yes, define the CNN once and pass it to both agent models via reference in the SKRL setup script to save VRAM.

## Environment Availability

| Dependency | Required By | Available | Version | Fallback |
|------------|------------|-----------|---------|----------|
| SKRL | MARL Policy | ✗ | — | Needs `pip install skrl` in Isaac Lab env |
| RTX 3060 (12GB) | HD Vision | ✓ | — | Ensure Early Pooling is aggressive |

**Missing dependencies with fallback:**
- `skrl` must be installed. Fallback is to remain on SB3 but limit to single-agent.

## Metadata

**Confidence breakdown:**
- Standard stack: HIGH - SKRL is the documented standard for Isaac Lab MARL.
- Architecture: HIGH - 5D tensor reshaping is standard PyTorch practice.
- Pitfalls: HIGH - VRAM is mathematically constrained by the 12GB limit; early pooling is mandatory.

**Research date:** 2026-05-11
**Valid until:** Late 2026 (Isaac Lab updates rapidly).
