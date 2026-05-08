# Phase 14-01: Multi-Agent Environment Refactor - Research Update

**Researched:** 2024-05-22
**Domain:** Isaac Lab `ManagerBasedRLEnv` Multi-Agent Configuration
**Confidence:** HIGH

## Summary
The investigation focused on implementing idiomatic Multi-Agent support within the `ManagerBasedRLEnv` framework using procedural (loop-based) configurations. While `DirectMARLEnv` is preferred for decentralized MARL, `ManagerBasedRLEnv` can effectively support multi-robot systems through centralized observation and action management. 

**Primary recommendation:** Use a `__post_init__` method or direct loop-based attribute assignment within `@configclass` to generate agent terms. Use `skrl` with its `isaaclab-multi-agent` wrapper to bridge the gap between centralized environment tensors and multi-agent policy updates.

## 1. Loop-based Configuration in `@configclass`
Isaac Lab's `@configclass` is a wrapper around Python `dataclasses`. It processes members at class definition time.

### Idiomatic Loop Pattern
The most reliable way to generate $N$ terms is within the `__post_init__` of the configuration class or by using `setattr` in the class body before the decorator finishes processing (though `__post_init__` is cleaner for dynamic counts).

```python
@configclass
class ObservationCfg:
    @configclass
    class PolicyCfg(ObservationGroupCfg):
        def __init__(self, num_agents: int = 2):
            super().__init__()
            for i in range(num_agents):
                # Using setattr to add ObservationTermCfg
                setattr(self, f"agent_{i}_pos", ObsTerm(
                    func=mdp.joint_pos_rel, 
                    params={"asset_cfg": SceneEntityCfg(f"robot_{i}")}
                ))
    
    policy: PolicyCfg = PolicyCfg()
```

## 2. Indexing `SceneEntityCfg`
`SceneEntityCfg` refers to entries in the `InteractiveSceneCfg`. 

- **Best Practice:** When spawning multiple robots in `SceneCfg` using a loop, ensure the names match the keys used in `ActionCfg` and `ObservationCfg`.
- **Pattern:**
```python
for i in range(num_agents):
    setattr(self, f"robot_{i}", ARTICULATION_CFG.replace(prim_path=f"{{ENV_REGEX_NS}}/Robot_{i}"))
```

## 3. Observation Tensor Dimensions
- **Behavior:** `ManagerBasedRLEnv` (via `ObservationManager`) expects terms to return tensors of shape `(num_envs, ...)`.
- **Concatenation:** If `concatenate_terms=True` (default in `ObservationGroupCfg`), all terms in a group are concatenated along the last dimension.
- **Flattening:** If a term returns `(num_envs, num_agents, obs_dim)`, the manager will concatenate it with other terms. If all terms have this shape, the final observation will be `(num_envs, num_agents, total_obs_dim)`. 
- **Constraint:** Most standard RL wrappers (like SB3 or basic `skrl`) expect `(num_envs, flattened_obs)`. To support true MARL, the `skrl` wrapper `isaaclab-multi-agent` should be used to reshape the `(num_envs, num_agents * obs_dim)` into `(num_envs * num_agents, obs_dim)` for the policy.

## 4. Known Issues with `setattr` and Dictionaries
- **Field Recognition:** `dataclasses` (and thus `configclass`) only recognizes variables with type annotations as fields. If using `setattr` inside `__init__`, these attributes are not technically "fields" of the dataclass but are present in the instance `__dict__`.
- **Manager Compatibility:** `ManagerBase` (the parent of Observation/Action managers) explicitly checks `self.cfg.__dict__.items()` if the config is not a dictionary. This means attributes added via `setattr` **will** be discovered by the managers even if they aren't formal dataclass fields.
- **Recommendation:** Always ensure the value assigned via `setattr` is a valid `ManagerTermBaseCfg` (like `ObsTerm` or `ActionTerm`).

## Environment Availability
| Dependency | Required By | Available | Version |
|------------|------------|-----------|---------|
| skrl | MARL Training | ✓ | 1.4.3 |
| Isaac Lab | Environment | ✓ | 0.24.x (approx) |

## Sources
- `isaaclab.managers.observation_manager` (Source code audit)
- `isaaclab.utils.configclass` (Source code audit)
- `Toni-SM/MARL_mav_carry_ext` (Reference implementation)
