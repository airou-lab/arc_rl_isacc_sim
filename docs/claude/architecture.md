# Architecture

This document describes the structure of the ARCPro RL stack as it currently exists in the repo: how Isaac Sim is launched, what each entry-point script does, the role of `arcproLab/`, how the `openStreetUSD/` assets are consumed, and the top-level data flow from USD asset to a trained PPO policy.

All claims below are sourced from the code as checked into the repo. Anything I could not verify is listed as an open question at the bottom.

## How Isaac Sim is launched

There is one Python entry point pattern, repeated by every script in `arcproLab/scripts/`:

1. The script builds an `argparse.ArgumentParser`, then calls `AppLauncher.add_app_launcher_args(parser)` (from `isaaclab.app`) so that Isaac Lab can inject its own CLI flags (`--headless`, `--enable_cameras`, etc.).
2. It parses args, then constructs `app_launcher = AppLauncher(args_cli)` and grabs `simulation_app = app_launcher.app`. This is what actually boots the Omniverse Kit application — the simulator process is alive once `AppLauncher(...)` returns.
3. Only *after* that do the scripts import the Isaac Lab Python modules they need (`ManagerBasedRLEnv`, USD utilities, etc.). This ordering matters: those imports depend on the Kit app being initialized.

The shell wrappers in the repo root do not launch Isaac Sim themselves. They shell out to Isaac Lab's launcher script at `/home/arika/IsaacLab/isaaclab.sh` with `-p <python_script>`, which runs the Python script inside Isaac Lab's bundled Python environment. So the chain is:

```
train.sh / verify_*.sh
  └─ /home/arika/IsaacLab/isaaclab.sh -p arcproLab/scripts/<script>.py
       └─ AppLauncher(...) inside the script → Omniverse Kit app
            └─ ManagerBasedRLEnv(cfg=ARCProEnvCfg()) → scene + managers
```

## Entry-point scripts

All four are thin shell wrappers around an `arcproLab/scripts/*.py` script run through Isaac Lab.

### `train.sh`
- Computes `PROJECT_DIR` from `BASH_SOURCE` (portable).
- Defaults: `NUM_ENVS=1`, no `--headless`, no checkpoint. Accepts `--num_envs`, `--headless`, `--checkpoint`.
- Invokes `arcproLab/scripts/train_policy.py` with the parsed flags **plus** a hard-coded `--total_timesteps 1000000` and `--enable_cameras`.

`train_policy.py` then:
1. Instantiates `ARCProEnvCfg`, sets `scene.num_envs` from CLI, forces `enable_cameras = True`, and re-runs `__post_init__()` so derived fields (decimation, episode length, optional observation pruning) are recomputed.
2. Builds the env: `ManagerBasedRLEnv(cfg=env_cfg)`.
3. Wraps it with `Sb3VecEnvWrapper` (Isaac Lab's SB3 adapter), then `VecNormalize(norm_obs=True, norm_reward=True, clip_obs=10.)`.
4. Creates a fresh `PPO("MlpPolicy", ...)` on CUDA, or loads `--checkpoint` via `PPO.load(...)` if supplied. PPO hyperparameters are hard-coded in the script (lr=3e-4, n_steps=2048, batch_size=64, n_epochs=10, γ=0.99, GAE λ=0.95, clip=0.2, ent_coef=0.0).
5. Logs to `logs/ppo/<YYYYmmdd-HHMMSS>/`. Three callbacks run every step: `CheckpointCallback` saves `model_*.zip` every 5000 steps, a local `SaveVecNormalizeCallback` writes `vec_normalize.pkl` every 5000 steps, and a `RewardLoggerCallback` prints a `[PROGRESS]` line every 1000 steps.
6. On completion saves `model_final.zip` and `vec_normalize.pkl`, then closes the env.

Note: the SB3 policy is `MlpPolicy`, not a CNN policy. The comment in `train_policy.py` explains that `Sb3VecEnvWrapper` flattens observations, so the camera image (if enabled) is concatenated with telemetry into a flat vector before reaching the network.

### `verify_sim.sh`
- Hard-codes `PROJECT_DIR=/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim` (not derived from `BASH_SOURCE` like the others).
- Invokes `arcproLab/scripts/verify_spawn.py --num_envs 1`. Does **not** pass `--headless`.

`verify_spawn.py` is a smoke test. It boots the env, calls `get_track_manager(device=env.device)`, resets, prints the initial robot position and the `compute_errors(...)` result (note: in the current `TrackManager` that returns zeros — see "TrackManager" below), then steps zero-action for 50 frames printing the robot position every 10 steps. Used to confirm the robot spawns without falling/clipping.

### `verify_sim_metric.sh`
- Computes `PROJECT_DIR` from `BASH_SOURCE`.
- Invokes `arcproLab/scripts/verify_metric.py --num_envs 1 --headless`.

`verify_metric.py` runs a closed-loop drive using `mdp/policy_wrapper.PolicyWrapper`, which loads a pre-trained CNN at `arcproLab/models/road_following_model.pth` — this is *not* the SB3 PPO model trained by `train.sh`. Each step it pulls the camera image out of `obs["visual"]`, calls `policy.predict(img)` → `(steering, throttle)`, builds a `(num_envs, 6)`-shaped action tensor (2 steering + 4 throttle slots), and steps the env. Used as a sanity check that physics, telemetry, and the legacy vision model all behave as expected at 1.0× metric scale. (The hard-coded action width of 6 here does not match the action manager's combined dim of 2 — see open questions.)

### `run_gui_verify.sh`
- Computes `PROJECT_DIR` from `BASH_SOURCE`.
- `find`s the most recently modified `*.zip` under `logs/ppo/`, errors out if none exists.
- Invokes `arcproLab/scripts/verify_live.py --checkpoint <latest> --num_envs 1`.

`verify_live.py` rebuilds the env exactly the way `train_policy.py` does, loads the matching `vec_normalize.pkl` next to the checkpoint, and rolls out the trained PPO policy in the GUI (not in the CLAUDE.md command summary, but this is the actual visual-verification script).

## `arcproLab/` — what lives where

`arcproLab/` is the project's Python package. Its top level holds the env configs and a few entry points; submodules hold the manager-callable MDP functions and supporting code.

- `arcpro_env_cfg.py` — the central `ARCProEnvCfg(ManagerBasedRLEnvCfg)`. It composes:
  - `ARCProSceneCfg` (lighting, the track USD, the robot, a tiled camera at `Chassis/CameraSensor`, a `CameraVisualCone` helper, and a `ContactSensorCfg` on the chassis),
  - `ObservationCfg` with two groups: `policy` (12-D telemetry via `mdp.observations.get_telemetry_vector`) and `visual` (the tiled camera RGB through `isaaclab.envs.mdp.image`),
  - `ActionCfg` with two action terms: `steering` and `throttle`, both using the custom `GroupedJoint*Action` classes from `mdp/actions.py`,
  - `RewardCfg` (5 terms: `speed`, `lateral_error`, `stationary` lambda, `heading`, `smoothness`),
  - `TerminationCfg` (4 terms: `height`, `roadmark_contact`, `stagnation`, `driving_blind`),
  - `EventCfg` (one term: `reset_robot_to_fixed_spawn`),
  - `SimulationCfg` at `dt=0.002` (500 Hz) with `render_interval=25` (→ 20 Hz visuals); `__post_init__` sets `decimation=25` so control runs at 20 Hz too, and `episode_length_s=120.0`.

- `arcpro_robot_cfg.py` — `ArcProRobotCfg(ArticulationCfg)`. Points at `arcproLab/assets/robot/F1Tenth_Metric.usd`, uses `mdp.spawner.spawn_f1tenth` (via a `spawn_f1tenth_preset` wrapper) to apply per-link mass overrides (Chassis 20 kg, Wheel_* 1 kg, Knuckle_* 0.1 kg). Defines two implicit actuator groups: `steering` (joints `Joint_Steer_.*`, stiffness 20, damping 2) and `throttle` (joints `Joint_Drive_.*`, stiffness 0, damping 1 — velocity-controlled).

- `arcproLab/__init__.py` — registers `gym.register("ARCPro-RL-v1", entry_point="isaaclab.envs:ManagerBasedRLEnv", kwargs={"cfg_entry_point": ARCProEnvCfg})`. (None of the entry-point scripts I read actually use the gym ID — they construct `ManagerBasedRLEnv(cfg=ARCProEnvCfg())` directly.)

- `mdp/` — the Isaac Lab manager system dispatches by string reference into these modules:
  - `observations.py` — `get_telemetry_vector` builds a 12-element tensor: relative odometry (X, Y, yaw), forward speed, yaw rate, last actions (steer, throttle), two slots that *would* be lat/heading error but are **hard-zeroed by design** — the chosen onboard sensor stack does not provide lateral/heading error, so the policy is forced to derive lane positioning from vision alone (the raw values are still written into `env.extras` for use by reward terms), and accumulated distance at index 11. Indices 6, 7, 10 are zero in the current code.
  - `rewards.py` — `speed_reward` (forward velocity, −2 penalty if reversing), `lateral_error_reward` (reads `env.extras["lat_err"]`; +1 inside 0.1 m, scaled penalty outside), `line_penalty` (−10 if `white_line_contact` triggered), `steering_jerk_penalty`, `action_rate_smoothness_reward` (penalises steering Δ), `heading_alignment_reward` (cosine of `env.extras["head_err"]`).
  - `terminations.py` — `white_line_contact` (proximity to any yellow/white marker < 0.1 m, computed by TrackManager), `fov_visibility_termination` (resets if drift angle > camera half-FOV after a 20-step settle window), `height_termination` (`z < 0.02` or `z > 0.5`), `stagnation_termination` (< 0.01 m progress for > 500 steps, tracked via `env.extras["distance"]`).
  - `actions.py` — `GroupedJointAction` and its position/velocity subclasses. Each term has `action_dim = 1` but broadcasts that scalar to multiple joints (so the policy emits one steering scalar that drives both steer joints, and one throttle scalar that drives all four drive joints). Optional `clip` is applied in `process_actions`; throttle is clipped to `(0.0, 1.0)` in the env cfg to force forward motion.
  - `events.py` — `reset_robot_to_fixed_spawn` teleports the robot to `(-16.25375, 5.56)` plus the per-env origin, raycasts down from z=100 to snap onto the road, sets yaw to `-π/2`, zeros root and joint velocities.
  - `track_manager.py` — singleton via `get_track_manager(device)`. On first sync it walks the USD stage under `/World/envs/env_0`, identifies any mesh whose path or material name contains `"yellow"` or `"white"`, transforms its vertices to world coords offset by the env-0 origin, dedupes to 2-cm precision, and uploads `yellow_tensor` / `white_tensor` to GPU. `compute_marker_distances(pos)` returns the per-env closest-marker distance via `torch.cdist`. `refresh_visuals()` draws debug spheres for the markers in the GUI. `compute_errors(...)` is currently a stub that returns zeros — `observations.py` still calls it and stores its (zero) outputs into `env.extras["lat_err"]` / `head_err`, which means `lateral_error_reward` and `heading_alignment_reward` are effectively constant under the current TrackManager.
  - `spawner.py` — `spawn_f1tenth` (USD spawn + per-link mass overrides via `schemas.define_mass_properties`) and `spawn_guide_cone` (cone marker with `purpose=guide` so sensors don't see it).
  - `policy_wrapper.py` — wrapper for the legacy `road_following_model.pth` (CNN), used by `verify_metric.py`.
  - `debug_terminations.py`, `visual_analytics.py` — debug helpers; `visual_analytics.TelemetryWindow` is the GUI overlay used by `verify_metric.py`.

- `assets/robot/` — the F1Tenth USD files: `F1Tenth_Metric.usd` (used) and `F1Tenth_Generated.usd` (not referenced by anything I read).

- `models/road_following_model.pth` — the legacy vision model loaded by `verify_metric.py` via `PolicyWrapper`. The SB3 PPO checkpoints go to `logs/ppo/` instead.

- `policy_stack/policies/` — `fusion_policy.py`, `hierarchical_policy.py`. These are mirrors of files that live in a separate "policy" repo under the broader ARCPro umbrella; they're not consumed by anything in *this* repo's training/verification entry points. Treat as out-of-scope copies for cross-repo reference.

- `scripts/` — entry points (`train_policy.py`, `verify_spawn.py`, `verify_metric.py`, `verify_live.py`) and a large family of one-off diagnostic / calibration scripts (`calibrate_lanes.py`, `measure_road_width.py`, `audit_*.py`, `diagnose_*.py`, etc.).

## How `openStreetUSD/` assets are consumed

The active track asset is `openStreetUSD/no_graph_sim_clean_1x.usda`. It is referenced exactly once in the live code, in `ARCProSceneCfg.track`:

```python
track = AssetBaseCfg(
    prim_path="{ENV_REGEX_NS}/Track",
    spawn=sim_utils.UsdFileCfg(
        usd_path=os.path.join(USD_DIR, "no_graph_sim_clean_1x.usda"),
        scale=(0.125, 0.125, 0.125),
    ),
    init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, 0.0)),
)
```

So Isaac Lab's `UsdFileCfg` spawns the track once per env under `{ENV_REGEX_NS}/Track`, with a 0.125× scale to bring the source-USD geometry down to 1.0× metric. The ground plane is intentionally *not* spawned — the surrounding terrain has been removed so an off-road robot falls into the void.

Once the track is in the stage, `TrackManager.collect_raw_marker_points()` walks all mesh prims under `/World/envs/env_0`, inspects each prim's USD path *and* its bound material's path for the substrings `"yellow"` / `"white"`, and harvests their vertex points. This is how lane-boundary geometry is extracted at runtime — there is no separate lane-data file. The harvested points are uploaded to `yellow_tensor` / `white_tensor` on GPU and used by `white_line_contact` termination.

Other files in `openStreetUSD/`:
- `no_graph_sim_1x.usda`, `no_graph_sim_final.usd`, `no_graph_sim.usd` — present but not referenced anywhere in the code I read.
- `archive/` — historical asset variants with a `README.md` documenting why each was retired.

There is also `arcproLab/mdp/track_centerline.npy` — a stored centerline waypoint array. It is *not* loaded by `TrackManager` in the current code; `TrackManager.wp_path` points at `track_centerline_1x.npy` (a file that does not exist in the repo). The actual `waypoints` attribute is initialised to a single hard-coded dummy point `[[-16.25, 5.56, -1.57]]`. See open questions.

## Top-level data flow

```
        +------------------------------------------------+
        | openStreetUSD/no_graph_sim_clean_1x.usda       |
        | arcproLab/assets/robot/F1Tenth_Metric.usd      |
        +-----------------------+------------------------+
                                |
                                | spawned via UsdFileCfg / ArticulationCfg
                                v
        +------------------------------------------------+
        | Isaac Sim (Omniverse Kit) stage                |
        |   /World/envs/env_*/Track                      |
        |   /World/envs/env_*/Robot                      |
        |   /World/envs/env_*/Robot/Chassis/CameraSensor |
        |   /World/envs/env_*/Robot/Chassis  [contact]   |
        +-----------------------+------------------------+
                                |
                                | InteractiveScene / ManagerBasedRLEnv
                                v
        +------------------------------------------------+
        | Managers run each control step (20 Hz)         |
        |                                                |
        |   Action manager:                              |
        |     PPO action (2,) ─→ GroupedJointPosition    |
        |     (steering, broadcast to 2 steer joints)    |
        |                     ─→ GroupedJointVelocity    |
        |     (throttle,  broadcast to 4 drive joints)   |
        |                                                |
        |   Observation manager:                         |
        |     get_telemetry_vector → (num_envs, 12)      |
        |     mdp.image(tiled_camera) → (N, 90, 160, 3)  |
        |                                                |
        |   TrackManager (lazy singleton):               |
        |     scans USD markers once, then               |
        |     compute_marker_distances on GPU each step  |
        |                                                |
        |   Reward manager: 5 terms, weighted sum        |
        |   Termination manager: 4 terms, OR'd           |
        |   Event manager: reset_robot_to_fixed_spawn    |
        +-----------------------+------------------------+
                                |
                                | Sb3VecEnvWrapper flattens obs dict
                                v
        +------------------------------------------------+
        | VecNormalize (running mean/std, clipped ±10)   |
        +-----------------------+------------------------+
                                |
                                v
        +------------------------------------------------+
        | Stable-Baselines3 PPO ("MlpPolicy", CUDA)      |
        |   action ∈ R^2  (steering, throttle)           |
        |   checkpoints → logs/ppo/<ts>/model_*.zip      |
        |   vec_normalize.pkl saved alongside            |
        +------------------------------------------------+
```

Per step:
1. PPO emits a 2-D action.
2. `Sb3VecEnvWrapper.step` un-flattens and forwards to `ManagerBasedRLEnv.step`.
3. The action manager dispatches to the two `GroupedJointAction` terms, which call `asset.set_joint_position_target` / `set_joint_velocity_target` on the matching joints.
4. PhysX advances 25 sim sub-steps (500 Hz / decimation 25 = 20 Hz control).
5. The observation manager calls `get_telemetry_vector` (which also updates `env.extras["distance"]`, `env.extras["lat_err"]`, `env.extras["head_err"]`, etc.) and `mdp.image` on the tiled camera.
6. Reward and termination managers run, reading from observations and `env.extras`. The event manager applies `reset_robot_to_fixed_spawn` for any env that terminated.
7. SB3 receives the flattened obs and the reward, the VecNormalize wrapper updates its running stats, PPO appends to its rollout buffer, and (every `n_steps=2048` rollout per env) PPO performs `n_epochs=10` of update.

## Open questions

### Resolved

- **Lateral/heading error masked to 0 in observations.** *Intentional.* The onboard sensor stack chosen for the real robot does not produce lateral- or heading-error signals, so the policy is forced to derive lane positioning from vision alone. Indices 8 and 9 in the telemetry vector are kept in the layout as zero-fill for protocol stability.
- **`policy_stack/policies/fusion_policy.py` and `hierarchical_policy.py`.** *Out of scope for this repo.* These files mirror counterparts in a separate "policy" repo under the broader ARCPro umbrella and are not consumed by anything in this directory.

### Open

1. **Should the `lateral_error` and `heading_alignment` reward terms also see zero?** The observation masking is by design (see above), but `lateral_error_reward` and `heading_alignment_reward` also read from `env.extras["lat_err"]` / `env.extras["head_err"]`, which are populated by `TrackManager.compute_errors` — currently a stub returning zeros. With weights 5.0 and 2.0, those terms emit a constant `+1 + cos(0) = +1 + 1` per step, contributing a fixed `+7.0/step` floor regardless of position. Two possibilities:
   - **(a)** The stub is dead/temporary and the rewards *should* be using true (privileged) sim-side error during training, with sensor-only inputs to the policy — a standard partial-obs RL setup. If so, restoring `compute_errors` to real computation is a TODO.
   - **(b)** The reward should also be sensor-only, in which case those two terms shouldn't be registered at their current weights and the survival-bonus floor is unintended.

2. **`TrackManager.wp_path` points at `track_centerline_1x.npy`**, which is not in this clone (only `track_centerline.npy` is). Most likely cause: the `_1x` file lives on the lab dev machine and was never committed. Note also that `wp_path` is set in `__init__` but never read in the current `TrackManager` — `self.waypoints` is hard-coded to a single dummy point — so even if the file were present, nothing would load it. So this is a likely-stale path *and* a likely-stale loader.

3. **`verify_metric.py` constructs an action tensor of shape `(num_envs, 6)`** (2 steering slots + 4 throttle slots) and steps the env with it, but the live `ActionCfg` expects shape `(num_envs, 2)`. Likely stale relative to the current `arcpro_env_cfg.py`. Unverified — worth running once to confirm whether it still works or errors out.

4. **`verify_sim.sh` hard-codes `PROJECT_DIR=/home/arika/...`** while the other three shell wrappers derive `PROJECT_DIR` from `BASH_SOURCE`. Likely a leftover from when the script was written on Arika's machine; portable-style would make it consistent with its siblings.

5. **`arcpro_env_cfg_temp.py`** sits next to `arcpro_env_cfg.py` but is never imported. Unknown — scratch file vs. planned variant.

6. **`generate_track.py`** at the package root is not invoked by any entry-point script. Unknown what it produces or whether it's still part of the asset-setup workflow.

7. **The gym registration `"ARCPro-RL-v1"`** in `arcproLab/__init__.py` is never used by the four entry-point scripts (they all construct `ManagerBasedRLEnv(cfg=ARCProEnvCfg())` directly). Unknown whether anything outside this repo imports it.
