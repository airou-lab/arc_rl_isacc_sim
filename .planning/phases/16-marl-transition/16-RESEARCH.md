# Phase 16: Multi-Agent Urban Coordination - Research (Audited)

**Researched:** 2026-05-11
**Domain:** Multi-Agent Reinforcement Learning (MARL), SKRL Integration, V2V Logic
**Confidence:** HIGH

## Summary

This research phase defines the transition from a single-agent simulation to a multi-agent urban coordination framework using **SKRL**. The audit of the initial Phase 16 plan identifies a misalignment with the SKRL decision (it incorrectly targeted SB3) and highlights the need for explicit V2V logic implementation in the Observation manager, a strict 16-camera VRAM budget (8 envs x 2 agents), and a "Reset-Aware" FIFO queue for intersection management to prevent deadlocks.

**Primary recommendation:** Replace all SB3-compatibility wrappers with native SKRL `Model` classes supporting 5D tensors, and implement the V2V relative coordinate math within the Observation manager to avoid environment-level overhead.

<user_constraints>
## User Constraints (from CONTEXT.md)

### Locked Decisions
- **Framework**: Use **SKRL** (MAPPO/IPPO) for MARL. (REPLACES SB3).
- **VRAM Budget**: Target **16 total cameras** (8 envs x 2 agents) to ensure stability on RTX 3060.
- **Physics**: 500Hz (`dt=0.002`), Control Rate: 20Hz.
- **Scale**: 1.0x Metric.
- **Action Fix**: Inverted Throttle (Scale: -60.0).

### the agent's Discretion
- V2V Communication vector design (1D).
- "Go Signal Precedence" and "Random Turn" logic in `RoadManager`.
- Refactoring `RoadGraph` and `TrackManager` for vectorization.

### Deferred Ideas (OUT OF SCOPE)
- Complex V2V communication protocols (focus on spatial coordination first).
- Full urban scene with foliage.
</user_constraints>

<phase_requirements>
## Phase Requirements

| ID | Description | Research Support |
|----|-------------|------------------|
| MARL-01 | Refactor RoadGraph to vectorized RoadManager with FIFO Queue. | Identified deadlock risk; proposed "Reset-Aware Token Clearing". |
| MARL-02 | Implement V2V Relative Coordinate Math in Observation Manager. | Pattern 2 from Phase 17 Research: `[rel_x, rel_y, rel_yaw, rel_vel_x, rel_vel_y, go_signal]`. |
| MARL-03 | SKRL-Native Model for 5D HD Vision Tensors. | Pattern 1 from Phase 17 Research: Reshaping `(B, N, C, H, W)` -> `(B*N, C, H, W)` for CNN. |
| MARL-04 | 500Hz Physics Stability Audit for Multi-Agent Collisions. | Ensure solver stability during robot-robot contacts. |
</phase_requirements>

## Standard Stack

### Core
| Library | Version | Purpose | Why Standard |
|---------|---------|---------|--------------|
| SKRL | 1.3+ | MARL Framework | Native GPU tensor support; excellent Isaac Lab integration. |
| Isaac Lab | 1.1.0+ | Simulation Engine | Industry standard for vectorized RL. |
| PyTorch | 2.2.0+ | DL Framework | Required for SKRL and CNN features. |

### Supporting
| Library | Version | Purpose | When to Use |
|---------|---------|---------|--------------|
| torch.nn.AdaptiveAvgPool2d | VRAM Mitigation | Essential | Crush HD vision (640x360) early in the pipeline. |

## Architecture Patterns

### Pattern 1: SKRL 5D Tensor Model
**What:** SKRL models must handle `(num_envs, num_agents, ...)` tensors. The `compute` method should flatten the batch for the CNN and restore it for the output.
**Example:**
```python
# arcproLab/policy_stack/models/skrl_fusion_model.py
class FusionMARLModel(GaussianMixin, Model):
    def compute(self, inputs, role):
        # inputs["observations"] contains (B, N, ...)
        # Flatten B*N for CNN, then unflatten for return.
```

### Pattern 2: Observation-Level V2V Math
**What:** Calculate relative positions and velocities of the nearest agent within the `mdp/observations.py` logic.
**Logic:**
- Use `torch.cdist` to find the nearest agent in the same environment.
- Transform relative vectors to the **ego-agent local frame**.

### Pattern 3: Reset-Aware FIFO Token Clearing
**What:** The `RoadManager` must clear an agent's token if that agent resets (e.g., crashes before completing the turn).
**Implementation:**
```python
# arcproLab/mdp/road_manager.py
def update(self, env):
    # ...
    # CRITICAL: If env.reset_buf[i] is True for an agent holding a token,
    # release the token immediately so the FIFO queue can advance.
    self.clear_tokens(env.reset_buf)
```

## Redlines for Phase 16 Plan

### Redline 1: Remove SB3 Compatibility (Plan 16-03)
- **Problem**: Task 3 in `16-03-PLAN.md` focuses on `MARLFlattenWrapper` for SB3.
- **Correction**: Replace with **Task 3: Implement SKRL Fusion Model**. This task should create the native SKRL `Model` class that handles 5D tensors directly, avoiding the need for a flattening wrapper that breaks MARL indexing.

### Redline 2: Adjust VRAM Target (Plan 16-03/04)
- **Problem**: Current plan targets 32 agents.
- **Correction**: Explicitly target **16 agents (8 envs x 2 agents)**. Update `arcpro_multi_env_cfg.py` and `stress_test_vram.py` to reflect this budget.

### Redline 3: V2V Logic Placement (Plan 16-01)
- **Problem**: `16-01-PLAN.md` does not specify where V2V math happens.
- **Correction**: Add a sub-task to Task 3: "Implement V2V relative coordinate math (relative pos/vel in local frame) inside `mdp/observations.py`".

### Redline 4: Deadlock Prevention (Plan 16-01)
- **Problem**: Missing "Reset-Aware Token Clearing".
- **Correction**: Add Task 4 to `16-01-PLAN.md`: **Implement Reset-Aware FIFO Queue**. Ensure `RoadManager` releases intersection tokens when an agent's `reset_buf` is triggered.

### Redline 5: Physics Stability (Plan 16-02)
- **Problem**: Missing audit for `dt=0.002` with 2-agent collisions.
- **Correction**: Add Task 4 to `16-02-PLAN.md`: **500Hz Physics Stability Audit**. Test high-speed collisions between robots to ensure the solver iterations are sufficient to prevent interpenetration or simulation instability.

## Common Pitfalls

### Pitfall 1: FIFO Deadlock
**What goes wrong:** Agent A enters the intersection, crashes, resets to the start of the track, but the `RoadManager` still thinks Agent A is "in the intersection". Agent B waits forever.
**Prevention:** Always check the `reset_buf` when updating the `RoadManager` state.

### Pitfall 2: Local vs World Frame Confusion
**What goes wrong:** V2V relative coordinates are provided in world coordinates.
**Prevention:** Transform `rel_pos` using `quat_rotate_inverse(ego_quat, rel_pos_world)`.

## Environment Availability

| Dependency | Required By | Available | Version | Fallback |
|------------|------------|-----------|---------|----------|
| SKRL | Policy Training | ✗ | — | `pip install skrl` |
| RTX 3060 | Simulation | ✓ | — | — |

## Sources

### Primary (HIGH confidence)
- `17-RESEARCH_MARL_STACK.md` - Framework decisions.
- `16-RESEARCH.md` - Initial multi-agent scoping.
- Isaac Lab TiledCamera Docs.

## Metadata

**Confidence breakdown:**
- SKRL Stack: HIGH (Validated by research)
- VRAM Budget: MEDIUM (Needs empirical testing)
- V2V Logic: HIGH (Standard relative math)

**Research date:** 2026-05-11
