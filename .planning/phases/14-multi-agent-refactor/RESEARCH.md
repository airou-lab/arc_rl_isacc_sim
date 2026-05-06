# Phase 14: Multi-Agent Environment Refactor - Research

**Researched:** 2024-05-22
**Domain:** Multi-Agent Reinforcement Learning (MARL) in Isaac Lab
**Confidence:** MEDIUM

## Summary
The current environment uses `ManagerBasedRLEnv` configured for a single robot instance per environment. To scale to Multi-Agent scenarios, we must decide between treating multiple robots as a single "multi-body" entity or using specialized MARL frameworks. Isaac Lab's `DirectMARLEnv` is the native way to handle decentralized agents, but `ManagerBasedRLEnv` can be adapted by expanding the observation and action managers to handle concatenated tensors for $N$ agents.

**Primary recommendation:** Use `skrl` with `IPPO` or `MAPPO` for training, as it has the most robust MARL support for Isaac Lab. Implement $N$ robot instances within each vectorized environment by indexing prim paths.

## Policy Stack Status
- **Verification:** Per user confirmation, the **Policy Stack** (`arcproLab/policy_stack/`) already supports multiple robots (via `WorkerScheduler` and `AgentNode` IDs).
- **Refinement:** No core changes are required in the policy stack for multi-agent support. The focus of Phase 14 remains on **Environment Scaffolding** (Spawning, Observation/Reward Managers, and Inter-Agent Collision detection).

## Design Intent & Scope
**Refined Strategy:** Unlike standard vectorized training where each environment is an isolated island with one robot, Phase 14 focuses on **Intra-Environment Multi-Agent (IEMA)** support. 
- **Goal:** Enable a single environment instance (e.g., one Smart Intersection) to contain **2+ robots** simultaneously.
- **Purpose:** To test and train coordination, yielding, and collision avoidance in a shared physical space.
- **Scaling:** We will still use `num_envs` for parallelism, but each environment will now have `num_agents_per_env >= 2`.

## MARL Design Decisions
- **Policy:** **Homogeneous Parameter Sharing**. Both robots in the intersection will share the same PPO network weights.
- **Reward:** **Individual Rewards**. Each robot is evaluated on its own lane-following and intersection success, though a mutual "collision penalty" will likely be added to discourage inter-agent crashes.
- **Termination:** Episodes may terminate for an agent on boundary hit OR on robot-robot collision. 

## Standard Stack
- **RL Framework:** `skrl` (v1.1.0+) - specifically for its `IPPO`/`MAPPO` implementations.
- **Environment Base:** `DirectMARLEnv` (Recommended for true MARL) or `ManagerBasedRLEnv` (with concatenated agents).
- **Communication:** Shared memory/Tensors via Isaac Lab's `Scene` buffers.

## Architecture Patterns

### Recommended Project Structure
- `arcproLab/marl/`: New directory for MARL-specific MDP terms.
- `arcproLab/arcpro_marl_env_cfg.py`: Specialized config for multi-agent setups.

### Multi-Agent Spawning Pattern
Modify `ARCProSceneCfg` to spawn robots using indexed paths:
```python
for i in range(num_agents):
    setattr(self, f"robot_{i}", ARCPRO_ROBOT_CFG.replace(
        prim_path=f"{{ENV_REGEX_NS}}/Robot_{i}",
        ...
    ))
```

## Don't Hand-Roll
| Problem | Don't Build | Use Instead |
|---------|-------------|-------------|
| Multi-agent synchronization | Custom buffers | `skrl` MARL wrappers |
| Observation concatenation | Manual loops | Isaac Lab `ObservationManager` with grouped terms |

## Common Pitfalls
1. **Camera Memory Exhaustion**: Each robot instance with a `TiledCamera` consumes significant GPU memory. Scaling to 32 envs with 4 agents each (128 cameras) may crash 8GB/12VRAM cards.
   - *Mitigation*: Lower resolution or use a single "Top-down" oracle camera for early MARL training.
2. **Action Inversion/Mapping**: Ensure action tensors are correctly sliced and sent to the respective robot joints.

## Implementation Strategy
1. **Wave 1: Scaffolding**: Refactor `arcpro_env_cfg.py` to allow a configurable `num_agents` parameter.
2. **Wave 2: Observation Mapping**: Update `mdp/observations.py` to return a tensor of shape `(num_envs, num_agents, obs_dim)`.
3. **Wave 3: MARL Training**: Integrate `skrl` to handle the multi-agent reward and policy update loop.
