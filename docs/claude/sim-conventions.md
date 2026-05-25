# Simulation Conventions

This document collects the cross-cutting conventions the ARCPro RL stack assumes everywhere: units, coordinate frames, USD prim layout, asset naming, scale factors, and the implicit contract between `openStreetUSD/` assets and the downstream code that reads them. All entries are sourced from the live code (`arcproLab/arcpro_env_cfg.py`, `arcproLab/arcpro_robot_cfg.py`, `arcproLab/mdp/*.py`).

## Units

Everything is **metric and radian** — no scaling factor lives outside the asset-spawn boundary.

| Quantity              | Unit                         | Examples / source                                                                                              |
| --------------------- | ---------------------------- | -------------------------------------------------------------------------------------------------------------- |
| Length / position     | metres                       | Robot spawn `(−16.25375, 5.56, 0.05)` (`arcpro_env_cfg.py:82`); marker proximity threshold `0.1 m` (`mdp/terminations.py:23-25`) |
| Linear velocity       | m/s                          | `root_lin_vel_b[:,0]` consumed directly as forward speed (`mdp/observations.py:36`); stationary threshold `0.5 m/s` (`arcpro_env_cfg.py:155`) |
| Angular velocity      | rad/s                        | `root_ang_vel_b[:,2]` as yaw rate (`mdp/observations.py:39`); throttle scale `60.0` → wheel ω ≤ 60 rad/s (`arcpro_env_cfg.py:144`) |
| Angle                 | radians                      | Spawn yaw `+π/2` (`mdp/events.py:24`) — heading along `+Y`, conforms to policy `IsaacDirectConfig.spawn_yaw` (OM 3.3-A); FOV half-angle `atan(2.65 / (2·1.93))` rad (`mdp/terminations.py:37`)    |
| Time                  | seconds                      | `sim.dt = 0.002` (500 Hz), `episode_length_s = 120.0` (`arcpro_env_cfg.py:191, 210`)                            |
| Mass                  | kilograms                    | Chassis 20 kg, Wheel 1 kg, Knuckle 0.1 kg (`arcpro_robot_cfg.py:16-20`)                                         |
| Image intensity       | `uint8 ∈ [0, 255]`           | `get_image_uint8` (`mdp/observations.py`) — conforms to policy `image` Box dtype (OM 1.3-B)                     |
| Distance accumulator  | metres                       | `env.extras["distance"] += root_lin_vel_b[:,0] * 0.05` (`mdp/observations.py:63`)                               |

The control-step dt of `0.05 s` in the distance accumulator is hard-coded — it matches the 20 Hz control rate (`decimation=25` at 500 Hz). If decimation ever changes, that constant must change with it.

## Coordinate frames

### World frame (`_w`)

USD stage coordinates. Z-up, right-handed.

- `asset.data.root_pos_w` — robot root position in metres (`mdp/observations.py:27`).
- `asset.data.root_quat_w` — robot root quaternion in **(w, x, y, z)** order (`mdp/observations.py:22-23`). The yaw extraction `atan2(2(qw·qz + qx·qy), 1 − 2(qy² + qz²))` is the standard Z-axis Euler formula for that quaternion ordering.
- Z-up is confirmed by `height_termination` (`mdp/terminations.py:50`): root z < 0.02 m → fell through the floor; z > 0.5 m → flipped/launched.

### Per-env origin frame

For parallel rollouts, Isaac Lab gives every env an origin offset stored in `env.scene.env_origins[env_id, :3]`. The training code converts world positions into per-env-local coordinates by subtracting that origin (`mdp/observations.py:26-27`, `mdp/events.py:26-31`, `mdp/track_manager.py:84-87`). All telemetry indices 0–2 (relative X, Y, yaw) are in this frame; the policy never sees absolute world coordinates.

`env_spacing = 50.0 m` (`arcpro_env_cfg.py:184`) — env origins are laid out on a grid with 50 m spacing, well clear of the ≈ 30 m track footprint.

### Robot body frame (`_b`)

Local frame attached to the chassis root, X-forward, Z-up.

- `root_lin_vel_b[:, 0]` is forward speed (the controller and rewards treat it as such, `mdp/observations.py:36`, `mdp/rewards.py:13`).
- `root_lin_vel_b[:, 1]` is lateral velocity — used by `fov_visibility_termination` to compute the drift angle `atan2(v_y, v_x)` (`mdp/terminations.py:40`).
- `root_ang_vel_b[:, 2]` is yaw rate (`mdp/observations.py:39`).

### Sensor frame

The tiled camera is attached as a child of the chassis (`{ENV_REGEX_NS}/Robot/Chassis/CameraSensor`, `arcpro_env_cfg.py:89`) with `convention="parent"` (`arcpro_env_cfg.py:96`). That means the camera offset is interpreted in the chassis's local frame:

- `pos = (0.28, 0.0, 0.16)` m — 28 cm in front of, 16 cm above the chassis origin.
- `rot = (1.0, 0.0, 0.0, 0.0)` — identity quaternion; the camera points along the chassis's local +X (forward).
- Image axes follow Isaac Lab's `TiledCamera` convention: tensor shape `(num_envs, height, width, channels) = (N, 90, 160, 3)` with origin at top-left, RGB channel order.

## Quaternion convention

Isaac Lab returns quaternions as **(w, x, y, z)**. Two pieces of code rely on this and would silently produce wrong angles if the order ever changed:

- `mdp/observations.py:22-23` — yaw extraction.
- `mdp/events.py:35-38` — building a Z-axis yaw quaternion as `(cos(yaw/2), 0, 0, sin(yaw/2))`.

The spawn quaternion `(0.7071, 0.0, 0.0, 0.7071)` (`arcpro_env_cfg.py:83`) is +90° about Z. The comment "+90 degrees Z-up" in that file is correct.

## Scale conventions

The project runs **at 1.0× metric scale** for the robot, but the source track USD was authored at a different scale and is scaled down at spawn:

| Asset                             | Authored scale         | Spawn scale                 | Result   |
| --------------------------------- | ---------------------- | --------------------------- | -------- |
| `F1Tenth_Metric.usd`              | already 1× metric      | `(1.0, 1.0, 1.0)`           | 1× metric |
| `openStreetUSD/no_graph_sim_clean_1x.usda` | 8× metric (legacy) | `(0.125, 0.125, 0.125)`    | 1× metric |

Source of the 0.125 factor: `ARCProSceneCfg.track` at `arcpro_env_cfg.py:60-68`. Note that the filename includes `_1x` and the comments call it the "clean 1x version," but the geometry inside the file is still at the historical 8× scale — the **`_1x` refers to the post-spawn intended scale**, not the authored scale. This naming has tripped people up before; the `(0.125, …)` scale is load-bearing.

CLAUDE.md captures the same fact: "Changing scale requires updating robot cfg, track USD scale, `track_centerline.npy`, and all distance thresholds in rewards/terminations."

A handful of legacy comments still document the 8×→1× conversion that was used to recompute spawn poses and viewer settings, e.g. `arcpro_env_cfg.py:81-82, 95-96, 180-181`. They are historical context, not active math.

## USD prim layout

The Isaac Lab `{ENV_REGEX_NS}` placeholder expands to `/World/envs/env_0`, `/World/envs/env_1`, … At runtime the stage looks like:

```
/World
├── light                                       (DistantLight, single global)
└── envs/
    ├── env_0/
    │   ├── Track/                               (referenced USD asset)
    │   │   └── … (road meshes + yellow/white markers)
    │   └── Robot/
    │       └── Chassis/                          (rigid root)
    │           ├── (drive + steer joints, wheels, knuckles)
    │           ├── CameraSensor                  (TiledCamera prim)
    │           └── CameraVisualCone              (debug cone, purpose=guide)
    ├── env_1/  …
    └── env_N/  …
/Visuals
├── YellowBoundaries/                            (debug markers, ALL envs share)
└── WhiteBoundaries/
```

A couple of conventions hang off this layout:

- **Per-env replication is the default.** Every asset whose `prim_path` starts with `{ENV_REGEX_NS}` is replicated under each env. `light` is the one exception — it sits outside the env namespace and is shared.
- **TrackManager scans `/World/envs/env_0` only** (`mdp/track_manager.py:57`). It assumes all envs are identical replicas, harvests yellow/white marker geometry from env 0, then uses the same point cloud (offset by each env's origin) for boundary checks across the whole batch.
- **Visual debug markers live under `/Visuals`** (`mdp/track_manager.py:120-122`), outside any env namespace. They are not part of any env's observation or collision world.

## Asset and prim naming patterns

### Robot joints

The action manager and the actuator config both rely on these exact names being present in `F1Tenth_Metric.usd`:

| Pattern                                   | Used by                                                            |
| ----------------------------------------- | ------------------------------------------------------------------ |
| `Joint_Steer_L`, `Joint_Steer_R`          | `ActionCfg.steering` (`arcpro_env_cfg.py:140`); `ImplicitActuatorCfg` regex `Joint_Steer_.*` (`arcpro_robot_cfg.py:57`) |
| `Joint_Drive_FL`, `Joint_Drive_FR`, `Joint_Drive_RL`, `Joint_Drive_RR` | `ActionCfg.throttle` (`arcpro_env_cfg.py:143`); regex `Joint_Drive_.*` (`arcpro_robot_cfg.py:64`); telemetry audit list in `verify_metric.py:90` |

### Robot links

Per-link mass overrides match these regexes (`arcpro_robot_cfg.py:16-20`):

| Regex          | Mass (kg) |
| -------------- | --------: |
| `Chassis`      |      20.0 |
| `Wheel_.*`     |       1.0 |
| `Knuckle_.*`   |       0.1 |

The contact sensor (`arcpro_env_cfg.py:116-121`) is attached specifically to `Chassis`. If the link were renamed, the contact sensor would silently stop firing.

### Drive direction convention

Throttle is clipped to `(0.0, 1.0)` and scaled by `60.0` → joint velocity target ∈ `[0, 60] rad/s` (`arcpro_env_cfg.py:144-145`). The clip is non-negative on purpose: **positive angular velocity on the drive joints = forward motion**, given the wheel-axis orientation in `F1Tenth_Metric.usd`. Anything that hand-commands the drive joints (e.g. ad-hoc verification scripts) must use positive values — see open question #3 in `architecture.md` where `verify_metric.py` uses negatives and is suspected stale.

### Lane-marker naming (the load-bearing one)

`TrackManager.collect_raw_marker_points()` (`mdp/track_manager.py:51-90`) walks every `UsdGeom.Mesh` prim under env 0, lower-cases both the prim path and the bound material's path, and classifies a mesh as:

- **yellow** if `"yellow"` appears anywhere in `path + mat_path`, or
- **white** if `"white"` appears anywhere in `path + mat_path`.

Anything else is ignored. This substring search is the entire interface between the track asset and the boundary-termination logic. **Any track USD shipped into this project must have lane markers whose prim or material name contains the literal substring `yellow` or `white` (case-insensitive).** New marker types (cyan? lane-cone?) won't be picked up without a code change.

### Visual marker prim paths

- `/Visuals/YellowBoundaries` and `/Visuals/WhiteBoundaries` — yellow/white debug spheres drawn by `TrackManager.refresh_visuals()` (`mdp/track_manager.py:120-122`).
- `{ENV_REGEX_NS}/Robot/Chassis/CameraVisualCone` — a cyan cone marking the camera's forward direction. It is spawned with `UsdGeomTokens.guide` so RenderProducts (i.e. the camera sensor itself) skip it (`mdp/spawner.py:14-27`).

## Spawning conventions

### Robot spawn point

Hard-coded in two places that must agree:

- The static `init_state.pos = (−16.25375, 5.56, 0.05)`, yaw `+π/2` in `ARCProSceneCfg.robot` (`arcpro_env_cfg.py:80-84`).
- The runtime reset in `mdp/events.py:22-24`: `(local_spawn_x, local_spawn_y) = (−16.25375, 5.56)`, yaw `+π/2`.

Both spawn paths now agree on `+π/2` (heading `+Y`), conforming to the policy repo's `IsaacDirectConfig.spawn_yaw = +1.5708` (OM 3.3-A). The previous mismatch (init-state `+π/2` vs reset-event `−π/2`) was resolved in V1 by flipping the reset event. Runtime verification: `arcproLab/scripts/check_spawn_yaw.py`.

### Surface snapping

`reset_robot_to_fixed_spawn` raycasts from `z=100 m` straight down (`mdp/events.py:46`) up to a length of 200 m to find the road surface. If a hit is found and the hit prim doesn't have `"robot"` in its path, the robot is placed at `hit_z + 0.1 m`. If nothing is hit, the robot falls from `z = env_origin_z + 0.1 m`.

This means downstream-loaded USDs **must have ray-castable collision geometry on the pavement** for the snap to work; meshes that are render-only will cause the robot to spawn in the air and then drop.

## `openStreetUSD/` — downstream contract

Anything dropped into `openStreetUSD/` and pointed at by `ARCProSceneCfg.track` must satisfy the following implicit contract:

1. **Spawned scale (0.125, 0.125, 0.125)** must yield 1× metric. If you re-author a track at native 1× metric, you must drop the 0.125 scale in `arcpro_env_cfg.py:64` to match.
2. **Lane markers** must be `UsdGeom.Mesh` prims with `yellow` or `white` in either the prim path or the bound material path (case-insensitive substring). No other detection mechanism exists.
3. **Vertex point counts** should be reasonable for a `torch.cdist` (`mdp/track_manager.py:96-97`) call against `(num_envs, 2)` — the current track produces a few hundred unique yellow/white points after `np.unique(np.round(..., 2))` dedup at 1 cm precision. Orders-of-magnitude more would slow termination checks.
4. **Pavement must be ray-castable from above** (collision-enabled) so `reset_robot_to_fixed_spawn`'s snap works (`mdp/events.py:41-52`).
5. **No ground plane required (or wanted).** The env intentionally omits a default ground plane (`arcpro_env_cfg.py:52-57`) so off-road robots fall into the void — this is what makes the "drive off the road → reset" loop work even when no termination fires. Adding a ground plane back would break that behaviour.
6. **Spawn coordinates `(−16.25375, 5.56)` must be on the road**, on a sensible lane, oriented to make `yaw = +π/2` (i.e. +Y in env-local frame) the correct "down the road" direction. If you re-pose or re-tile the track, both this constant *and* the viewer eye/lookat in `arcpro_env_cfg.py:181` need to move with it.
7. **Mesh prims live under `/World/envs/env_0/<...>`** after spawn. `TrackManager` only walks env 0; it does not scan other envs or `/World` directly.

Other files in `openStreetUSD/`:

- `no_graph_sim.usd`, `no_graph_sim_final.usd`, `no_graph_sim_1x.usda` — present in the directory but not referenced by the env config. The historical README inside `archive/` claims `no_graph_sim_final.usd` is "the stable version," but the live code points at `no_graph_sim_clean_1x.usda` instead.
- `archive/` — historical asset variants with their own README documenting why each was retired. Not for live use.

## What's deliberately *not* a convention

A few things look like conventions but are local quirks worth knowing about:

- **The `_1x` in filenames means "intended-1× after spawn scaling," not "authored at 1×."** Geometry inside `no_graph_sim_clean_1x.usda` is at the legacy 8× scale.
- **The 12-element telemetry layout** is referred to as a "legacy protocol" in `mdp/observations.py:14`. Indices 6, 7, 8, 9, 10 are zero-fill / masked-by-design slots; the layout is preserved for protocol stability, not because all twelve slots carry information.
- **`env.extras` as a side channel** between observation, reward, and termination managers (e.g. `extras["distance"]`, `extras["lat_err"]`, `extras["stagnant_steps"]`, `extras["last_dist"]`) — this is how MDP terms share derived state without recomputing it. New MDP code in this project follows the same pattern.
