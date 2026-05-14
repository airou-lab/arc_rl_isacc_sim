# Phase 18: Advanced MARL Coordination - Research

**Researched:** 2026-05-11
**Domain:** Multi-Agent Reinforcement Learning (MARL), Coordination, Reward Shaping, V2V Transform
**Confidence:** HIGH

## Summary

This research focuses on the coordination mechanics required to evolve the ARCPro single-agent policy into a multi-agent system capable of navigating intersections. With a strict **12GB VRAM budget** and **HD Vision** requirements, the architecture must balance centralized learning with memory efficiency. **MAPPO (Centralized Training, Decentralized Execution)** is recommended due to its ability to handle non-stationarity in intersection scenarios, provided that aggressive VRAM mitigation (Early Pooling + Shared Encoders) is applied.

**Primary recommendation:** Use MAPPO with a shared Vision Backbone across all agents. Initialize the MARL transition by loading SB3 weights into a custom SKRL Model and freezing the CNN layers to focus initial learning on V2V-based coordination logic.

<user_constraints>
## User Constraints (from CONTEXT.md)

### Locked Decisions
- Target: Milestone 4 (MARL Coordination).
- Constraint: 12GB VRAM (RTX 3060).
- Scale: 8 environments x 2 agents.
- Logic: "Warm start" from current single-agent SB3 policy.

### the agent's Discretion
- Choice between MAPPO and IPPO.
- Reward shaping coefficients for Social Momentum.
- Coordinate transform implementation details.
- Transition strategy for weight loading.

### Deferred Ideas (OUT OF SCOPE)
- More than 2 agents per environment.
- Communication latency modeling.
- Dynamic lane changing (beyond intersection navigation).
</user_constraints>

## 1. MARL Critics: CTDE vs. Independent

### MAPPO (Centralized Training, Decentralized Execution)
- **Mechanism:** A centralized critic ($V_{shared}(s, o_1, o_2)$) sees the joint state or concatenated observations of all agents during training, while the actor ($\pi(a|o_{local})$) only uses local info at inference.
- **Pros:** Directly addresses the non-stationarity problem (where Agent A's learning environment changes because Agent B is also learning). Crucial for intersections where one agent's action (stopping) is only "correct" if the other moves.
- **Cons:** Larger rollout buffer.

### IPPO (Independent PPO)
- **Mechanism:** Each agent treats others as part of the environment.
- **Pros:** Minimal memory overhead; simple implementation.
- **Cons:** High risk of "selfish" behavior or failure to converge in high-conflict scenarios like unsignaled intersections.

### VRAM Budget Analysis (12GB)
For 8 envs x 2 agents:
- **Baseline (Isaac Lab + RTX):** ~5.0 GB.
- **Rollout Buffer (MAPPO, 2048 steps):** 
  - If storing 640x360 float32: **OOM (90GB+)**.
  - If storing 128x128 uint8 (Adaptive Pool): **~1.6 GB**.
  - Centralized Critic overhead (Joint State): **+0.2 GB**.
- **Neural Networks:** Shared ResNet/NatureCNN backbone + small MLP heads: **~1.0 GB**.
- **Total Estimate:** **~7.8 GB**.
- **Conclusion:** MAPPO is **HIGHLY FEASIBLE** for the 2-agent case on 12GB.

**Recommendation:** **MAPPO**. The coordination benefits far outweigh the minor memory cost of the joint critic for $N=2$.

## 2. Reward Shaping for Coordination

To prevent the "Parking Deadlock" (both agents stopping indefinitely out of fear of collision), we propose a tiered reward structure:

### Collision vs. Stationary Ratio
- **Collision Penalty ($R_{coll}$):** $-50.0$ to $-100.0$. Must be severe enough to discourage clipping.
- **Stationary Penalty ($R_{stat}$):** $-0.05$ per step if $|velocity| < 0.1 m/s$.
- **Critical Ratio:** $R_{coll} / R_{stat} \approx 1000:1$. If the ratio is too high, agents will park. If too low, they will "yolo" through the intersection.

### "Social Momentum" Reward
Reward the group for **Collective Throughput**:
- $R_{social} = \alpha \sum_{i=1}^{N} (v_{i, progress})$
- **Logic:** If Agent A yields so Agent B can pass, the *total* reward for Agent A remains positive because Agent B is making progress. This creates a "selfless" incentive to clear the intersection quickly as a unit.
- **Recommended $\alpha$:** $0.5$ (initial).

### Anti-Deadlock Strategies
1. **Asymmetric Observations:** Include the `go_signal` (FIFO-based) from the RoadManager in the V2V vector.
2. **Commitment Reward:** Extra $+0.1$ for maintaining velocity above $0.5 m/s$ once inside the intersection box.

## 3. V2V Frame Transform Performance

The agent must see the peer in its own **Local Ego Frame**.

### Implementation Comparison
| Method | Complexity | Vectorization | Recommendation |
|--------|------------|---------------|----------------|
| `torch.matmul(R_inv, p_rel)` | $O(N)$ | Good | Medium. Requires $q \to R$ conversion. |
| `math_utils.quat_rotate_inverse` | $O(1)$ | Excellent | **HIGH**. Uses specialized quaternion formula. |

### Optimized PyTorch Pattern
```python
from isaaclab.utils import math as math_utils

# p_ego, q_ego: (B, N, 3), (B, N, 4)
# p_peer: (B, N, 3) (swapped indices)

# 1. World-frame relative vector
rel_pos_world = p_peer - p_ego

# 2. Rotate into ego frame using conjugate of ego_quat
# quat_rotate_inverse performs: v' = q^-1 * v * q
rel_pos_local = math_utils.quat_rotate_inverse(q_ego, rel_pos_world)

# 3. Repeat for velocities
rel_vel_local = math_utils.quat_rotate_inverse(q_ego, v_peer - v_ego)
```

## 4. Transfer Learning (SB3 -> SKRL)

### Weight Loading Strategy
SB3 saves models as a zip containing a `policy.pth` (state_dict). SKRL uses standard PyTorch but a different model structure.

**Mapping Table:**
| SB3 Key (NatureCNN) | SKRL Key (Custom Model) |
|---------------------|-------------------------|
| `features_extractor.cnn.0.*` | `cnn.0.*` |
| `features_extractor.cnn.2.*` | `cnn.2.*` |
| `features_extractor.cnn.4.*` | `cnn.4.*` |
| `features_extractor.linear.0.*` | `cnn_fc.*` |

### Transition Protocol
1. **Wave 0:** Instantiate `SKRL.Model` with identical CNN architecture.
2. **Wave 1:** Load SB3 state_dict, stripping the `features_extractor.` prefix.
3. **Wave 2:** **Freeze Backbone**: `for param in model.cnn.parameters(): param.requires_grad = False`.
4. **Wave 3:** Train only the MLP heads and the new V2V input layers for 1M steps.
5. **Wave 4:** Unfreeze and fine-tune.

## Don't Hand-Roll

| Problem | Don't Build | Use Instead | Why |
|---------|-------------|-------------|-----|
| Centralized Critic | Custom joint-state MLP | SKRL `MAPPO` | Handles value-loss calculation for multi-agent tensors natively. |
| Frame Transforms | Manual rotation matrices | `math_utils.quat_rotate_inverse` | Specifically optimized for Isaac/PyTorch tensor shapes. |
| Weight Mapping | Manual layer assignment | `model.load_state_dict(..., strict=False)` | Allows loading vision weights while ignoring missing V2V/Policy heads. |

## Common Pitfalls

### Pitfall 1: Symmetric Deadlock
**What goes wrong:** Both agents stop at the intersection and never move.
**Why it happens:** Identical rewards and symmetric positions lead to identical gradient updates.
**How to avoid:** Use the `RoadManager` to provide an asymmetric `go_signal` (0 or 1) based on arrival time.

### Pitfall 2: HD Vision VRAM Leak
**What goes wrong:** Training crashes after 10 minutes.
**Why it happens:** Failure to clear the rollout buffer or storing full-res frames.
**How to avoid:** Use `AdaptiveAvgPool2d((128, 128))` as the very first operation in the model `compute()` to crush memory usage before it reaches the rollout buffer storage.

## Code Examples

### SKRL Model with Frozen Vision + V2V
```python
class MARLFusionModel(GaussianMixin, Model):
    def __init__(self, ...):
        super().__init__(...)
        self.cnn = nn.Sequential(...) # NatureCNN
        self.v2v_net = nn.Linear(6, 32) # [rel_x, rel_y, rel_yaw, rel_vx, rel_vy, go]
        self.fc = nn.Linear(512 + 32, 256)
        
    def compute(self, inputs, role):
        # inputs["observations"]: (B, N, C, H, W)
        x = self.cnn(inputs["observations"]["image"].view(-1, C, H, W))
        v2v = self.v2v_net(inputs["observations"]["v2v"].view(-1, 6))
        
        fused = torch.cat([x, v2v], dim=-1)
        return self.policy_head(fused).view(B, N, -1), ...
```

## Sources

### Primary (HIGH confidence)
- **SKRL Documentation:** MAPPO implementation details and multi-agent tensor handling.
- **Isaac Lab Math Utils:** `quat_rotate_inverse` source code.
- **SB3 Documentation:** `NatureCNN` state_dict key structure.

### Secondary (MEDIUM confidence)
- **MARL Research (IPPO vs MAPPO):** Coordination benchmarks in multi-agent driving tasks.

## Metadata

**Confidence breakdown:**
- Standard stack: HIGH - SKRL + Isaac Lab is a stable pairing.
- Architecture: HIGH - MAPPO is well-understood for N=2.
- Pitfalls: HIGH - VRAM is the primary known constraint.

**Research date:** 2026-05-11
**Valid until:** 2026-06-11
