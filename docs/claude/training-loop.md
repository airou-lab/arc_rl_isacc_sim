# Training Loop

This document specifies the RL training loop as wired in the code: observation space, action space, reward function, reset conditions, episode length, and the env config keys with their types and defaults. Numbers, shapes, and defaults are pulled directly from `arcproLab/arcpro_env_cfg.py`, `arcproLab/arcpro_robot_cfg.py`, and the `arcproLab/mdp/` modules.

## Observation space

Two observation groups are declared in `ObservationCfg` (`arcpro_env_cfg.py:123-134`):

### `policy` group — 12-D telemetry vector

Produced by `mdp_obs.get_telemetry_vector` (`mdp/observations.py:10-94`).

- **Shape**: `(num_envs, 12)`
- **dtype**: `torch.float32` (allocated via `torch.zeros(..., device=env.device)` at `mdp/observations.py:18`)
- **Bounds**: unbounded raw floats. `VecNormalize` in the training script then clips normalised values to ±10 (`scripts/train_policy.py:66`).

Component-by-component:

| Idx | Meaning                                    | Source                                                        | Notes                                                  |
| --- | ------------------------------------------ | ------------------------------------------------------------- | ------------------------------------------------------ |
| 0   | Relative X (m) since env origin            | `asset.data.root_pos_w[:,0] - env.scene.env_origins[:,0]`     | `mdp/observations.py:27,30`                            |
| 1   | Relative Y (m) since env origin            | `asset.data.root_pos_w[:,1] - env.scene.env_origins[:,1]`     | `mdp/observations.py:27,31`                            |
| 2   | Current yaw (rad)                          | `atan2(2(qw·qz + qx·qy), 1 − 2(qy² + qz²))`                   | `mdp/observations.py:23,32`                            |
| 3   | Forward speed (m/s)                        | `asset.data.root_lin_vel_b[:,0]` (body-frame X)               | `mdp/observations.py:36`                               |
| 4   | Yaw rate (rad/s)                           | `asset.data.root_ang_vel_b[:,2]`                              | `mdp/observations.py:39`                               |
| 5   | Last steering action (post-scale)          | `env.action_manager.action[:,0]`                              | `mdp/observations.py:43-45`; guarded by try/except     |
| 6   | Last throttle action — **stays 0**         | The slice is `[:, 5:7]`, but `action_manager.action.shape[1]` is 2, so index 6 is the throttle | `mdp/observations.py:43-45` |
| 7   | Unused — always 0                          | Tensor is zero-initialised and never written                  | `mdp/observations.py:18`                               |
| 8   | Lateral error — **hard-zeroed by design**  | `obs[:, 8] = 0.0`                                             | `mdp/observations.py:70`. Onboard sensor stack does not provide this; vision-only by design |
| 9   | Heading error — **hard-zeroed by design**  | `obs[:, 9] = 0.0`                                             | `mdp/observations.py:71`. Same reason as idx 8           |
| 10  | Unused — always 0                          | Zero-initialised, never overwritten                           | `mdp/observations.py:18`                               |
| 11  | Accumulated distance travelled (m)         | Running sum of `root_lin_vel_b[:,0] * 0.05` per step          | `mdp/observations.py:63,74-75`                         |

NaNs in the result are zeroed before return (`mdp/observations.py:88-92`).

The (intended-to-be-unmasked) lateral and heading errors are written to `env.extras["lat_err"]` and `env.extras["head_err"]` for the reward terms to consume (`mdp/observations.py:66-67`). The observation masking at indices 8/9 is a deliberate sensor-realism constraint — the real onboard sensor stack doesn't produce those signals — but in the current code the *extras* are also zero because `TrackManager.compute_errors` is a stub returning zeros (`mdp/track_manager.py:136-138`). That collapses the lateral and heading reward terms to constants; whether that was intended is open — see `architecture.md` open question #1.

### `visual` group — RGB camera

Produced by `isaaclab.envs.mdp.image` with `normalize=True` (`arcpro_env_cfg.py:132`), reading from the `tiled_camera` sensor configured at `arcpro_env_cfg.py:88-98`.

- **Sensor**: `TiledCameraCfg` mounted at `{ENV_REGEX_NS}/Robot/Chassis/CameraSensor`
- **Resolution**: `width=160, height=90`
- **Channels**: `data_types=["rgb"]` → 3
- **Shape**: `(num_envs, 90, 160, 3)` per Isaac Lab's tiled-camera convention
- **dtype**: `torch.float32`, normalised to `[0.0, 1.0]` by `normalize=True`
- **Optics**: `horizontal_aperture=2.65`, `focal_length=1.93` → half-FOV ≈ `atan(2.65 / (2·1.93)) ≈ 0.602 rad ≈ 34.5°` (full ≈ 69°)
- **Offset**: `(0.28, 0.0, 0.16)` m relative to the chassis, identity rotation

The `visual` group is set to `None` and the camera sensor is removed if `enable_cameras=False` (`arcpro_env_cfg.py:213-215`).

### Flattening for SB3

`Sb3VecEnvWrapper` flattens the dict of obs groups into a single vector before reaching `MlpPolicy` (`scripts/train_policy.py:63,84`). With cameras enabled, that's `12 + 160·90·3 = 43 212` floats per env per step. The wrapper-output is then run through `VecNormalize(norm_obs=True, ..., clip_obs=10.)`.

## Action space

Two action terms in `ActionCfg` (`arcpro_env_cfg.py:138-146`), both implemented by the custom `GroupedJointAction` classes at `mdp/actions.py:22-105`.

| Term       | Class                            | `action_dim` | Joints                                                       | Scale | Offset | Clip          |
| ---------- | -------------------------------- | -----------: | ------------------------------------------------------------ | ----: | -----: | ------------- |
| `steering` | `GroupedJointPositionAction`     |            1 | `Joint_Steer_L`, `Joint_Steer_R`                             |   1.0 |    0.0 | (default ±1)  |
| `throttle` | `GroupedJointVelocityAction`     |            1 | `Joint_Drive_RL`, `Joint_Drive_RR`, `Joint_Drive_FL`, `Joint_Drive_FR` |  60.0 |    0.0 | `(0.0, 1.0)`  |

- **Combined action shape**: `(num_envs, 2)`
- **dtype**: `torch.float32`
- **Bounds**: `steering ∈ [−1.0, +1.0]` (from `GroupedJointAction.action_space`, `mdp/actions.py:46-58`), `throttle ∈ [0.0, 1.0]` (from the `clip={"throttle": (0.0, 1.0)}` entry on the throttle cfg, `arcpro_env_cfg.py:145`).
- **Semantics**: a single steering scalar is broadcast to both steering joints; a single throttle scalar is broadcast to all four drive joints. After scale/offset, steering becomes a joint position target in radians (`set_joint_position_target`, `mdp/actions.py:96`); throttle becomes a velocity target up to 60 rad/s (`set_joint_velocity_target`, `mdp/actions.py:105`).
- **Actuators**: implicit, from `arcpro_robot_cfg.py:55-69`. Steering: stiffness 20, damping 2, effort 5 N·m. Throttle: stiffness 0, damping 1, effort 5 N·m, velocity limit 100 rad/s.

## Reward function

Five terms in `RewardCfg` (`arcpro_env_cfg.py:148-161`), all weighted-summed by Isaac Lab's reward manager each control step.

| Term                  | Function                                             | Weight | What it does                                                                 | File / lines                            |
| --------------------- | ---------------------------------------------------- | -----: | ---------------------------------------------------------------------------- | --------------------------------------- |
| `speed`               | `speed_reward`                                       |   5.0  | `+0.5 · vₓ` when `vₓ > 0`, **−2.0 flat** when `vₓ ≤ 0`                       | `mdp/rewards.py:10-22`                  |
| `lateral_error`       | `lateral_error_reward`                               |   5.0  | Reads `env.extras["lat_err"]`. `+1` if `|err| < 0.1 m`, else `−5·|err|`      | `mdp/rewards.py:24-45`                  |
| `stationary`          | inline `lambda env: where(root_lin_vel_b[:,0] < 0.5, −5.0, 0.0)` |   1.0  | Flat `−5` per step whenever forward speed is under 0.5 m/s                   | `arcpro_env_cfg.py:154-157`             |
| `heading`             | `heading_alignment_reward`                           |   2.0  | `cos(env.extras["head_err"])`, NaNs zeroed                                   | `mdp/rewards.py:79-90`                  |
| `smoothness`          | `action_rate_smoothness_reward`                      |   1.0  | `−1 · (steeringₜ − steeringₜ₋₁)²`; returns 0 if `prev_action` not yet set    | `mdp/rewards.py:61-77`                  |

Two more reward functions exist in `mdp/rewards.py` but are **not registered** in `RewardCfg` and are therefore inactive: `line_penalty` (`mdp/rewards.py:47-52`, would emit −10 on `white_line_contact`) and `steering_jerk_penalty` (`mdp/rewards.py:54-59`).

`VecNormalize(norm_reward=True)` then maintains a running variance and normalises the summed reward before PPO sees it (`scripts/train_policy.py:66`).

### Caveat on current behaviour

Because `env.extras["lat_err"]` and `env.extras["head_err"]` are always 0 under the current `TrackManager.compute_errors` stub, `lateral_error` reliably returns `+1.0` per step (the `|0| < 0.1` branch) and `heading` reliably returns `cos(0) = 1.0`. Their weighted contribution is therefore a constant `5.0 + 2.0 = 7.0` floor as long as the env is alive. The training signal in practice comes from `speed`, `stationary`, and `smoothness` plus that survival bonus.

## Reset conditions

Four termination terms in `TerminationCfg` (`arcpro_env_cfg.py:163-175`). Any one of them being `True` for an env triggers a reset of that env via the `EventCfg.reset_robot_to_fixed_spawn` event (`arcpro_env_cfg.py:32-39`).

| Term              | Function                          | Condition                                                                                                                                          | File / lines                  |
| ----------------- | --------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------- |
| `height`          | `height_termination`              | `root z < 0.02 m` (fell through floor) or `root z > 0.5 m` (flipped / launched)                                                                    | `mdp/terminations.py:46-50`   |
| `roadmark_contact`| `white_line_contact`              | Robot centre within `0.1 m` of any yellow or white marker vertex (computed on GPU by `TrackManager.compute_marker_distances`)                       | `mdp/terminations.py:10-32`   |
| `stagnation`      | `stagnation_termination`          | Accumulated `env.extras["distance"]` has increased by less than `0.01 m` for more than `500` consecutive control steps (25 s at 20 Hz)              | `mdp/terminations.py:52-61`   |
| `driving_blind`   | `fov_visibility_termination`      | After a 20-step settle window, if `|atan2(v_y, v_x)| > 0.602 rad` (camera half-FOV) **and** planar speed > 0.5 m/s                                  | `mdp/terminations.py:34-44`   |

On reset, `reset_robot_to_fixed_spawn` (`mdp/events.py:14-61`) teleports the robot to `(−16.25375, 5.56)` plus the env origin, raycasts from `z=100` down to snap onto the road surface (then `+0.1 m`), sets yaw to `−π/2`, and zeros root + joint velocities.

There is no time-limit *truncation* term registered, but Isaac Lab itself enforces the `episode_length_s` timeout — see below.

## Episode length

Set in `ARCProEnvCfg.__post_init__` (`arcpro_env_cfg.py:208-215`):

```python
self.decimation = 25            # one control step = 25 sim sub-steps
self.episode_length_s = 120.0   # max episode wall-clock seconds
```

Combined with `SimulationCfg(dt=0.002)` (`arcpro_env_cfg.py:191`):

- Sim rate: `1 / 0.002 = 500 Hz`
- Control rate: `500 / 25 = 20 Hz`
- Max steps per episode: `120.0 · 20 = 2400` control steps

`render_interval = 25` (`arcpro_env_cfg.py:192`) keeps the GUI at the same 20 Hz cadence.

## Env config keys — types and defaults

All defined on `ARCProEnvCfg(ManagerBasedRLEnvCfg)` (`arcpro_env_cfg.py:177-215`):

| Key                | Type                  | Default                                                                                              |
| ------------------ | --------------------- | ---------------------------------------------------------------------------------------------------- |
| `viewer`           | `ViewerCfg`           | `ViewerCfg(eye=(-15.0, 6.875, 1.25), lookat=(-16.25, 5.56, 0.0))`                                    |
| `enable_cameras`   | `bool`                | `True`                                                                                               |
| `scene`            | `ARCProSceneCfg`      | `ARCProSceneCfg(num_envs=32, env_spacing=50.0)`                                                      |
| `observations`     | `ObservationCfg`      | `ObservationCfg()` — `policy` + `visual` (`visual` removed when `enable_cameras=False`)             |
| `actions`          | `ActionCfg`           | `ActionCfg()` — `steering` (scale 1.0), `throttle` (scale 60.0, clip `(0.0, 1.0)`)                  |
| `rewards`          | `RewardCfg`           | `RewardCfg()` — five terms above                                                                     |
| `terminations`     | `TerminationCfg`      | `TerminationCfg()` — four terms above                                                                |
| `events`           | `EventCfg`            | `EventCfg()` — single `reset_robot_to_fixed_spawn` term, mode `"reset"`                              |
| `sim`              | `SimulationCfg`       | `dt=0.002`, `render_interval=25`, `device="cuda:0"`, PhysX TGS solver, position iters 16, velocity iters 4, CCD on, stabilization on, GPU contact/patch/heap caps (`arcpro_env_cfg.py:190-206`) |

Derived in `__post_init__` (`arcpro_env_cfg.py:208-215`):

| Key                | Type    | Value                                       |
| ------------------ | ------- | ------------------------------------------- |
| `decimation`       | `int`   | `25`                                        |
| `episode_length_s` | `float` | `120.0`                                     |
| `viewer.camera_follow_prim_path` | `str \| None` | `None` (camera does not follow) |
| `observations.visual` | `VisualCfg \| None` | `None` if `enable_cameras=False`, else as declared |
| `scene.tiled_camera`  | `TiledCameraCfg \| None` | `None` if `enable_cameras=False`             |

Notable defaults inside `ARCProSceneCfg` (`arcpro_env_cfg.py:42-121`):

| Key                       | Type                | Default                                                                                                  |
| ------------------------- | ------------------- | -------------------------------------------------------------------------------------------------------- |
| `light`                   | `AssetBaseCfg`      | `DistantLightCfg(intensity=3000.0, color=(1,1,1))`                                                       |
| `track`                   | `AssetBaseCfg`      | USD `openStreetUSD/no_graph_sim_clean_1x.usda`, scale `(0.125, 0.125, 0.125)`, pos `(0,0,0)`             |
| `robot`                   | `ArcProRobotCfg`    | USD `arcproLab/assets/robot/F1Tenth_Metric.usd`, scale `(1,1,1)`, spawn pos `(−16.25375, 5.56, 0.05)`, yaw `+π/2` |
| `tiled_camera`            | `TiledCameraCfg`    | 160×90 RGB, aperture 2.65, focal 1.93, offset `(0.28, 0, 0.16)`                                          |
| `camera_cone`             | `AssetBaseCfg`      | Cyan cone (radius 0.03, height 0.1, opacity 0.5) marked `purpose=guide` so cameras don't see it          |
| `contact_forces`          | `ContactSensorCfg`  | Chassis contact sensor, history length 3, `debug_vis=True`                                               |

And in `ArcProRobotCfg` (`arcpro_robot_cfg.py:23-70`):

| Key                                  | Type / Value                                                                                                                                            |
| ------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `spawn.usd_path`                     | `arcproLab/assets/robot/F1Tenth_Metric.usd`                                                                                                             |
| `spawn.scale`                        | `(1.0, 1.0, 1.0)`                                                                                                                                       |
| `spawn.activate_contact_sensors`     | `True`                                                                                                                                                  |
| `spawn.rigid_props`                  | gravity on, damping 0, max lin/ang vel 1000, max depenetration 100                                                                                      |
| `spawn.articulation_props`           | self-collisions off, position iters 8, velocity iters 4, root not fixed                                                                                  |
| `init_state.pos`                     | `(0.0, 0.0, 0.1)` (overridden in scene cfg)                                                                                                              |
| `init_state.joint_pos`               | `{".*": 0.0}`                                                                                                                                            |
| `actuators["steering"]`              | `ImplicitActuatorCfg` on `Joint_Steer_.*`, effort 5, vel-limit 10, stiffness 20, damping 2                                                              |
| `actuators["throttle"]`              | `ImplicitActuatorCfg` on `Joint_Drive_.*`, effort 5, vel-limit 100, stiffness 0, damping 1                                                              |
| Mass overrides (in `spawn_f1tenth_preset`, `arcpro_robot_cfg.py:14-21`) | `Chassis=20.0 kg`, `Wheel_.*=1.0 kg`, `Knuckle_.*=0.1 kg`                                                            |

## End-to-end step ordering

1. **PPO** emits a `(num_envs, 2)` action.
2. **`Sb3VecEnvWrapper`** un-flattens and calls `ManagerBasedRLEnv.step(action)`.
3. **Action manager** splits the action into the two terms; each `GroupedJointAction.process_actions` applies its scale + clip and broadcasts the scalar across its joints; `apply_actions` calls `set_joint_position_target` (steering) and `set_joint_velocity_target` (throttle) (`mdp/actions.py:68-105`).
4. **PhysX** advances `decimation=25` sub-steps at `dt=0.002` (one control step = 50 ms wall-clock).
5. **Observation manager** runs `get_telemetry_vector` (which also accumulates `env.extras["distance"]` and writes `lat_err`/`head_err`) and `mdp.image(tiled_camera)`.
6. **Reward manager** computes the five weighted terms and sums them.
7. **Termination manager** ORs the four termination terms.
8. **Event manager** runs `reset_robot_to_fixed_spawn` for every env whose termination fired (or whose `episode_length_s` was exceeded — Isaac Lab handles that internally).
9. **`VecNormalize`** updates obs/reward running stats and clips obs to ±10.
10. PPO writes the transition into its rollout buffer; once `n_steps=2048` per env has accumulated, it runs `n_epochs=10` of mini-batch updates (`batch_size=64`, `clip_range=0.2`, `gamma=0.99`, `gae_lambda=0.95`, `ent_coef=0.0`, `learning_rate=3e-4`) before resuming rollouts.
