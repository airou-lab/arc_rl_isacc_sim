# Phase 11 Debug Log: Environment & Policy Alignment

This document records the extensive "plumbing" issues encountered and resolved while integrating the `HierarchicalPathPlanningPolicy` (HPPP) with Isaac Lab's `ManagerBasedRLEnv`. These are classic examples of "Technology Clash" between high-performance simulation frameworks and stable RL libraries.

## 1. The "Technology Clash": Observation Keys
- **Issue**: Isaac Lab's `Sb3VecEnvWrapper` is hardcoded to expect an observation group named `policy`. However, the HPPP Contract requires keys named `vec` and `image`.
- **Symptom**: `KeyError: 'policy'` when wrapping the environment.
- **Resolution**: Reverted `arcpro_env_cfg.py` observation group names back to `policy` and `visual` to satisfy Isaac Lab. Implemented a custom `HPPPDirectBridge` (inheriting from `VecEnv`) that intercepts the observation dictionary and maps `policy` -> `vec` and `visual` -> `image` just before the data reaches the policy.

## 2. Flattened Single-Term Observation Groups
- **Issue**: Attempting to access `obs["policy"]["telemetry"]` caused crashes.
- **Symptom**: `TypeError: 'Box' object is not subscriptable`.
- **Cause**: Isaac Lab aggressively optimizes observation groups. If a group (like `policy`) contains only a single term (`telemetry`), Isaac Lab removes the nested dictionary structure and outputs a flat tensor. 
- **Resolution**: Updated `HPPPDirectBridge._process_obs()` to stop "drilling down." We now directly assign the flattened tensor: `"vec": obs["policy"]`.

## 3. Asynchronous `VecEnv` Initialization
- **Issue**: `ManagerBasedRLEnv` is synchronous, but SB3's `VecEnv` base class requires asynchronous step methods.
- **Symptom**: `AttributeError: 'ManagerBasedRLEnv' object has no attribute 'step_async'`.
- **Resolution**: Fully implemented the `VecEnv` interface in `HPPPDirectBridge`. We "fake" the async behavior by storing the action tensor in `step_async()` and executing `self.base_env.step()` during `step_wait()`.

## 4. HWC to CHW Broadcast Errors (Image Transposition)
- **Issue**: Isaac Lab outputs camera images in `(Height, Width, Channels)` format. PyTorch CNNs expect `(Channels, Height, Width)`.
- **Symptom**: `ValueError: operands could not be broadcast together with shapes (3,90,160) (160,1,90)` during Normalization.
- **Cause**: When attempting to infer the correct shape for the new observation space, accessing `img_space.shape` from the batched environment returned `(1, 90, 160, 3)`. The manual transposition using indices `[2], [0], [1]` resulted in a corrupted shape `(160, 1, 90)`.
- **Resolution**: Updated the bridge to read `single_observation_space` from the environment to get the unbatched `(90, 160, 3)` shape. Manually applied `np.transpose(img, (0, 3, 1, 2))` during observation processing to correctly format the batch for PyTorch `(B, C, H, W)`.

## 5. Action Space Bounds Missing
- **Issue**: Isaac Lab's default ActionManager produces an unbounded continuous action space `(-inf, inf)`. SB3's PPO requires strict bounds to calculate probabilities.
- **Symptom**: `AssertionError: Continuous action space must have a finite lower and upper bound`.
- **Resolution**: Explicitly overrode the `action_space` property inside `HPPPDirectBridge` using `gym.spaces.Box(low=-1.0, high=1.0, shape=(3,))` to enforce the `[steer, throttle, brake]` limits.

## 6. Robot Moving Backwards
- **Issue**: The robot moved backwards despite positive throttle commands (e.g., `Local X Speed: -0.260`).
- **Cause**: During the implementation of `CombinedDriveAction`, the `scale` parameter in `arcpro_env_cfg.py` was mistakenly set to `-60.0`. This was based on a comment from an older Isaac ROS2 environment script, but `ManagerBasedRLEnv` driving the F1Tenth model expects a positive scale for forward movement.
- **Resolution**: Reverted `scale` back to `60.0` in `CombinedDriveActionCfg` within `arcpro_env_cfg.py`.

---
*Status: All plumbing issues resolved. The HPPPDirectBridge correctly translates Isaac Lab's high-performance, synchronous, flattened, unbounded, HWC data into the structured, asynchronous, strictly bounded, CHW format required by the sb3_contrib RecurrentPPO policy.*