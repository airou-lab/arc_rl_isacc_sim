# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ARCPro RL is a reinforcement learning system for vision-based lane following on an F1Tenth differential-drive robot in NVIDIA Isaac Sim / Isaac Lab. The robot operates at 1.0x metric scale (real-world dimensions) for sim-to-real transfer. The RL environment is built on Isaac Lab's `ManagerBasedRLEnv`.

## Commands

**Training:**
```bash
./train.sh --num_envs 16 --headless --total_timesteps 1000000
# Core script: arcproLab/scripts/train_policy.py
# Checkpoints saved to: logs/ppo/{timestamp}/model_*.zip
```

**Verification:**
```bash
./verify_sim.sh          # Spawn/physics checks (headless)
./run_gui_verify.sh      # Visual verification with latest trained model
./verify_sim_metric.sh   # Metric validation
```

**Linting:**
```bash
flake8 arcproLab/
```

**Tests:**
```bash
pytest tests/test_track_manager.py
```

**Single test:**
```bash
pytest tests/test_track_manager.py::TestClassName::test_method_name -v
```

**Dependencies:** Install via `pip install -r requirements.txt`. Requires Isaac Lab at `/home/arika/IsaacLab/isaaclab.sh` and PyTorch with CUDA.

## Architecture

### Environment Configuration (`arcproLab/arcpro_env_cfg.py`)
Central config for the Isaac Lab `ManagerBasedRLEnv`. Registers all managers (observations, rewards, terminations, actions, events) and sets physics parameters: 500Hz sim, 20Hz control, PhysX TGS solver, GPU pipeline.

### MDP Modules (`arcproLab/mdp/`)
The Isaac Lab manager system dispatches to these modules:
- `observations.py` — 12D telemetry (PVP-masked layout, V1 reunification). Optional camera RGB (224×224×3, uint8).
- `rewards.py` — Speed (5.0×), lateral error (5.0×), heading alignment (2.0×), action smoothness (−1.0×), stationary penalty (−5.0).
- `terminations.py` — Height bounds, white-line contact (0.1m threshold), stagnation (500 steps), FOV visibility.
- `actions.py` — `GroupedJointPositionAction` (steering) and `ThrottleBrakeVelocityAction` (throttle + brake folded into a 2-D term; brake multiplicatively attenuates throttle, matching the policy's `isaac_direct_env` semantics).
- `track_manager.py` — GPU-accelerated waypoint and lane-boundary tracking using USD marker detection. Critical for computing lateral/heading errors.
- `events.py` — Reset and initialization logic (robot pose, joint states).
- `spawner.py` — Asset spawning utilities.

### Robot (`arcproLab/arcpro_robot_cfg.py`)
F1Tenth at 1.0x scale: 0.45m width, 20kg chassis, 1kg wheels. USD: `F1Tenth_Metric.usd`. Two steering joints (position), four drive joints (velocity). Camera: 224×224 RGB uint8, ~69° FOV, mounted on chassis.

### Track Asset
`openStreetUSD/no_graph_sim_clean_1x.usda` with 0.125 USD scale factor (yields 1.0x metric). Yellow and white lane markers define boundaries. Centerline waypoints at `arcproLab/mdp/track_centerline.npy`.

### Training Pipeline
`train_policy.py` wraps the Isaac Lab env in a Stable-Baselines3 `VecNormalize` wrapper and trains with PPO on CUDA. The env runs 16–32 parallel instances.

### Observation Vector Layout

Two observation groups are registered in `ObservationCfg` (`arcproLab/arcpro_env_cfg.py:123-134`).

**`vec` group — 12-D telemetry**, built by `get_telemetry_vector` (`arcproLab/mdp/observations.py`).
- Shape: `(num_envs, 12)`, dtype: `torch.float32`, allocated on `env.device`.
- V1 reunification (2026-05-25): layout conforms to the policy repo's `TELEMETRY_INDICES`. The sim has no Worker / Scheduler, so slots 0/1 ship `0.0` (a checkpoint trained against `IsaacDirectEnv` reads slot 0 as `turn_token ∈ {-1, 0, 1}` — `rel_x` would be off-distribution).
- All values are raw metric / radian (see `docs/claude/sim-conventions.md`).

| Idx | Meaning                              | Unit  | Source                                                                                              |
| --- | ------------------------------------ | ----- | --------------------------------------------------------------------------------------------------- |
| 0   | turn_token (PVP, sim has no Worker)  | —     | `0.0`                                                                                               |
| 1   | go_signal (PVP, sim has no Scheduler)| —     | `0.0`                                                                                               |
| 2   | goal_dist (PVP-masked)               | m     | `0.0`                                                                                               |
| 3   | Forward speed (body X)               | m/s   | `root_lin_vel_b[:,0]`                                                                               |
| 4   | Yaw rate (body Z)                    | rad/s | `root_ang_vel_b[:,2]`                                                                               |
| 5   | last_steer                           | —     | `action_manager.action[:,0]`                                                                        |
| 6   | last_throttle                        | —     | `action_manager.action[:,1]`                                                                        |
| 7   | last_brake (brake actuator deferred) | —     | `0.0` (OM 2-B)                                                                                      |
| 8   | lat_err (PVP-masked)                 | m     | `0.0`                                                                                               |
| 9   | head_err (PVP-masked)                | rad   | `0.0`                                                                                               |
| 10  | reserved (PVP)                       | —     | `0.0`                                                                                               |
| 11  | Accumulated distance travelled       | m     | running sum of `root_lin_vel_b[:,0] * 0.05` (cumulative across resets; OM 1.2-C deferred)            |

Raw `lat_err` / `head_err` from `TrackManager.compute_errors` still flow into `env.extras` for the reward function. NaNs in the obs vector are zeroed before return.

**`visual` group — camera RGB**, produced by `isaaclab.envs.mdp.image(..., normalize=True)` (`arcproLab/arcpro_env_cfg.py:132`), sourced from the `tiled_camera` sensor (`arcproLab/arcpro_env_cfg.py:88-98`).
- Shape: `(num_envs, 224, 224, 3)` (H, W, C — Isaac Lab `TiledCamera` convention).
- dtype: `torch.uint8`, values in `[0, 255]` (V1 reunification: matches policy `Box(0, 255, uint8)` declaration, OM 1.3-B). Produced by `mdp/observations.py::get_image_uint8`.
- Resolution `224 × 224`, `data_types=["rgb"]`, horizontal aperture 2.65, focal length 1.93 → ≈ 69° horizontal FOV.
- Camera offset relative to chassis: `(0.28, 0.0, 0.16) m`, identity rotation, `convention="parent"`.
- The visual group and the camera sensor are both set to `None` when `enable_cameras=False` (`arcpro_env_cfg.py:213-215`).

`Sb3VecEnvWrapper` flattens the dict into a single vector before the SB3 `MlpPolicy` sees it (`scripts/train_policy.py:63, 84`). With cameras enabled, that is `12 + 224·224·3 = 150 540` values per env per step.

### Action Vector Layout

Two action terms in `ActionCfg` (`arcproLab/arcpro_env_cfg.py`), implemented by `GroupedJointPositionAction` (steering) and `ThrottleBrakeVelocityAction` (throttle + brake) in `arcproLab/mdp/actions.py`.

- Shape: `(num_envs, 3)`, dtype: `torch.float32`. V1 reunification: brake channel matches policy contract and now actuates as a multiplicative throttle derate. Independent brake torque is still deferred to V2.
- Action layout: `[steering, throttle, brake]`. Steering is 1-D and broadcasts across the two steering joints; throttle+brake is a 2-D term that fuses into one wheel velocity command per drive joint as `vel = 60.0 * throttle * (1 - brake)`.

| Idx | Name      | Range          | Unit (post-scale)                      | Class                                                       | Joints driven                                                                            |
| --- | --------- | -------------- | -------------------------------------- | ----------------------------------------------------------- | ---------------------------------------------------------------------------------------- |
| 0   | steering  | `[−1.0, +1.0]` | rad (joint position target, scale 1.0) | `GroupedJointPositionAction`                                | `Joint_Steer_L`, `Joint_Steer_R`                                                          |
| 1   | throttle  | `[ 0.0, +1.0]` | rad/s (joint velocity target, scale 60.0) | `GroupedJointVelocityAction`                              | `Joint_Drive_FL`, `Joint_Drive_FR`, `Joint_Drive_RL`, `Joint_Drive_RR`                    |
| 2   | brake     | `[ 0.0, +1.0]` | derate factor `(1 - brake)` applied to throttle command | `ThrottleBrakeVelocityAction` (folded into throttle term)  | `Joint_Drive_FL`, `Joint_Drive_FR`, `Joint_Drive_RL`, `Joint_Drive_RR`                    |

- Steering bound comes from `GroupedJointAction.action_space` default (`mdp/actions.py:46-58`).
- Throttle and brake bounds come from per-channel clipping inside `ThrottleBrakeVelocityAction.process_actions` (`mdp/actions.py`). Both channels are clamped to `[0, 1]`; throttle is non-negative on purpose (positive ω on drive joints = forward motion given the wheel-axis orientation in `F1Tenth_Metric.usd`).
- Final command sent to PhysX: `set_joint_position_target(scale·a + offset)` for steering, `set_joint_velocity_target(scale·a + offset)` for throttle (`mdp/actions.py:85-105`).

### Checkpoint Format

Training writes to `logs/ppo/<YYYYmmdd-HHMMSS>/` (`scripts/train_policy.py:69`). Three artefacts land in that directory:

| File                         | Producer                                                | Cadence                              |
| ---------------------------- | ------------------------------------------------------- | ------------------------------------ |
| `model_<step>_steps.zip`     | SB3 `CheckpointCallback(save_freq=5000, name_prefix="model")` (`train_policy.py:130`) | every 5000 env steps                |
| `vec_normalize.pkl`          | Local `SaveVecNormalizeCallback(save_freq=5000)` (`train_policy.py:119-128, 131`)     | every 5000 env steps (same cadence) |
| `model_final.zip` + `vec_normalize.pkl` | Saved at end of training (`train_policy.py:142-144`)                                       | once                                |

**Yes — `VecNormalize` statistics are saved alongside the SB3 model.** They are *not* embedded in the `.zip`; they live in the sibling `vec_normalize.pkl`. To resume or roll out a checkpoint, both files are required and must come from the same training run.

- Loading at training resume: `PPO.load(args_cli.checkpoint, env, ...)` (`train_policy.py:73-82`). The `vec_normalize.pkl` is *not* automatically loaded here — restarts currently re-fit the running stats from scratch (an open issue worth flagging).
- Loading at inference time: `run_gui_verify.sh` picks the latest `.zip` under `logs/ppo/` (`run_gui_verify.sh`) and `verify_live.py` then loads the matching `vec_normalize.pkl` from the same directory.

Action / observation shapes baked into a checkpoint: action `(3,)` (steer, throttle, brake — brake unwired per OM 2-B), observation flattened size `12 + 224·224·3 = 150 540` when `enable_cameras=True`, or `12` when disabled. A checkpoint trained with one of those configurations is not interoperable with the other.

## Key Design Decisions

- **1.0x metric scale**: The entire stack (robot, track, waypoints, reward thresholds) assumes real-world metric units. Changing scale requires updating robot cfg, track USD scale, `track_centerline.npy`, and all distance thresholds in rewards/terminations.
- **Termination vs. reward**: Hard boundary violations use terminations; soft lane-centering uses reward shaping. The `white_line_contact` termination uses a 0.1m center-to-marker threshold.
- **Manager-based RL**: Isaac Lab's manager system calls MDP functions by string reference from the env cfg. Adding a new reward/termination requires both a function in `mdp/` and a `RewardTermCfg`/`TerminationTermCfg` entry in `arcpro_env_cfg.py`.
- **GPU-first**: All tensor operations in MDP modules must stay on the simulation device (`env.device`). Avoid CPU↔GPU transfers in the hot path.

## Docs
- `docs/PROJECT.md` — Vision and milestone roadmap (v1.0–v1.2)
- `docs/STATE.md` — Current training phase and next steps
- `docs/REQUIREMENTS.md` — Requirement traceability matrix
- `docs/claude/architecture.md` — How Isaac Sim is launched and the top-level data flow
- `docs/claude/training-loop.md` — Obs / action / reward / reset / config specifics
- `docs/claude/sim-conventions.md` — Units, frames, USD layout, naming, `openStreetUSD/` contract
- `.planning/` — Per-phase planning documents

## Detailed context

The following files auto-load into every Claude Code session in this directory:

@docs/PROJECT.md
@docs/STATE.md
