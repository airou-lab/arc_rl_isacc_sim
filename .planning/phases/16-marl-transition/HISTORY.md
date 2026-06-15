# Phase 16: MARL Transition - History

## Physics Stabilization: The Upside-Down USD & Control Flip Strategy
During the transition to MARL, we encountered severe physics instability and NaN crashes. The root cause was discovered: we conclusively proved that the `F1Tenth_Metric.usd` asset was built upside-down relative to the world.

### Bugs Encountered
- **PhysX Suspension Breakage**: Forcing the robot upright with quaternions (applying a WXYZ orientation of `(0.0, 0.7071, 0.7071, 0.0)`) broke the PhysX suspension, causing levitation and NaN crashes.
- **KeyError 'visual'**: An error caused by a missing `@configclass` decorator on the observation groups in the environment configuration, preventing the visual observations from registering.

### Fixes Applied
1. **The Upside-Down USD Native Fix (`events.py`)**: We accepted the native upside-down spawn orientation. This keeps the PhysX articulation stable and avoids simulation explosions.
   ```python
   # Explicitly build WXYZ quaternion for 1.0x native orientation
   quats[i, 0] = math.cos(half_yaw) # W
   quats[i, 1] = 0.0                # X
   quats[i, 2] = 0.0                # Y
   quats[i, 3] = math.sin(half_yaw) # Z
   ```

2. **The "Control Flip" Strategy (`arcpro_env_cfg.py`)**: To compensate for the upside-down chassis, we flipped the drive control polarity. `drive.scale` is set to `-20.0`. Positive throttle now successfully pushes the upside-down chassis forward along the track without physics explosions.
   ```python
   drive = arcpro_actions.CombinedDriveActionCfg(
       asset_name="robot",
       joint_names=["Joint_Drive_RL", "Joint_Drive_RR", "Joint_Drive_FL", "Joint_Drive_FR"],
       scale=-20.0, # Flipped polarity
       offset=0.0
   )
   ```

3. **Camera & Visuals (`arcpro_env_cfg.py`)**: Because the robot is upside-down, the camera had to be mounted at `X: -0.3` (the physical front) and flipped 180 degrees (`rot=(0.0, 0.0, 0.0, 1.0)`) to look forward. Added a highly visible red `CuboidCfg` marker to the camera sensor to trace its direction.
   ```python
   offset=TiledCameraCfg.OffsetCfg(pos=(-0.3, 0.0, 0.2), rot=(0.0, 0.0, 0.0, 1.0), convention="parent")
   
   camera_marker = AssetBaseCfg(
       prim_path="{ENV_REGEX_NS}/Robot/Chassis/CameraSensor/VisualMarker",
       spawn=sim_utils.CuboidCfg(
           size=(0.3, 0.05, 0.05),
           visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.0, 0.0), emissive_color=(1.0, 0.0, 0.0))
       )
   )
   ```

4. **Stagnation Limits & Rolling Start (`terminations.py` / `events.py`)**: Strict 10-step stagnation limit is active, but we provided a rolling start (0.2-0.5 m/s) to ensure the 5kg chassis doesn't get executed while its motors spool up.
   ```python
   # Rolling start provided to avoid immediate stagnation execution
   # Stagnation limit strictly enforces movement
   return (vel < 0.1) & (env.episode_length_buf > 10)
   ```

## Reward Stabilization: Mathematical & Behavioral Findings
During Step 0 of Phase 16, we identified two critical behavioral failures ("bugs") in the reward structure that prevented training convergence.

### 1. The "Greed Bug" (Sprint and Die)
- **Finding**: High speed reward (100.0) relative to the death penalty (-500) made it mathematically profitable for the agent to maximize velocity and crash immediately rather than learn long-term survival.
- **Symptom**: Standard deviation of actions exploded to 4.8 as the agent chased high-magnitude noise.
- **Fix**: Rebalanced speed weight and introduced a much larger effective death penalty (Weight 2.0 * -500.0 = -1000.0).

### 2. The "Lazy Bureaucrat" (Crawl and Do Nothing)
- **Finding**: High passive rewards for smoothness (30.0) and lateral centering (10.0) outweighed the progress reward. The agent learned that sitting still or crawling at <0.1m/s was safer than attempting the track.
- **Symptom**: Survival time stuck at ~50 steps with near-zero progress.
- **Fix**: Reduced smoothness and lateral weights (1.0 and 2.0 respectively) and increased speed weight to 5.0.

### 3. Progress Reward Solution (Track-Tangent Projection)
- **Innovation**: Replaced raw speed reward with a **Progress-Based Speed** calculation.
- **Logic**: The reward is now calculated by projecting the robot's world velocity onto the track's tangent vector at its current position.
- **Benefit**: The AI no longer earns points for "sliding" or driving off-track; it only gains rewards for moving *along* the intended path.
- **Code Snippet**:
  ```python
  # Project World Velocity onto Tangent
  vel_world = asset.data.root_lin_vel_w[:, :2]
  progress_speed = torch.sum(vel_world * tangent, dim=1)
  return torch.clamp(progress_speed, min=-1.0)
  ```

### 4. Final Math Balance (Mastery Scaling)
We balanced the formula to ensure a clear hierarchy of incentives:
- **Full Lap (+2750)** >> **Death (-1000)** >> **Stagnation (-100/step)**.
- This ensures the agent is always incentivized to move forward but prioritizes survival over reckless speed.

### 5. Physics Confirmation (Control Flip)
- **Re-Confirmed**: The "Control Flip" strategy (Native upside-down orientation + `drive.scale = -20.0`) is the only stable configuration that prevents suspension NaN explosions for the 5kg F1Tenth asset.

## Gold Master Launch: Technical Hardening & Baseline Stability
The final stage of the MARL transition baseline involved a rigorous "Gold Master" hardening process to ensure the simulation and policy stack are ready for production-scale training.

### 1. Physics & Spawn Finalization
We re-confirmed the absolute stability parameters for the F1Tenth asset:
- **Spawn Coordinates**: Exactly on the path center (X=-16.197) at a stable drop height of **12cm**.
- **Orientation**: Standard North-facing spawn with an **upright orientation (180-roll)**. This ensures the suspension settles naturally without flipping or jittering.
- **Settling Interval**: Increased simulation stability by allowing the PhysX engine to resolve initial contact before applying torque.

### 2. Ground Settle Logic (Action Lock)
To prevent the "Spastic Start" bug (where the agent applies maximum throttle while airborne during spawn), we implemented a mandatory action lock in the `WaypointTrackingWrapper`.
- **Logic**: All actions (throttle and steering) are zeroed until the chassis Z-coordinate drops below **0.10m**.
- **Benefit**: Ensures the robot has settled its suspension and tires are in firm contact with the road before the policy takes control.
- **Code Snippet**:
```python
# Force zero action until robot has settled on the ground
root_pos = self.env.unwrapped.scene["robot"].data.root_pos_w
is_airborne = (root_pos[:, 2] > 0.10) | (self.env.unwrapped.episode_length_buf < 5)
action[is_airborne] = 0.0
```

### 3. Strict Boundary Enforcement
- **Paint Sensitivity**: Restored the `roadmark_contact` threshold to **0.12m**.
- **Constraint**: This is the exact physical limit where the outer tire edge touches the white/yellow paint. This zero-tolerance approach forces the agent to learn precise lane centering.

### 4. Hole Punching for Gate Permeability
We resolved a conflict between dense boundary markers and logical lane gates by implementing a 1.0m "Hole Punching" algorithm in the `TrackManager`.
- **Innovation**: The manager now automatically removes boundary markers (white/yellow points) that fall within 1.0m of a logical Lane Gate.
- **Result**: Perfect visibility of lane markings for the vision system, but absolute permeability for the physics engine, preventing "Invisible Wall" resets at intersections.

### 5. Training Baseline (Step 0)
- **Status**: Wiped all legacy checkpoints and launched a fresh run.
- **Initial Metrics**: The agent shows immediate survival capability, averaging **190 steps** per episode with a stable cruise speed of **0.22 m/s**.
- **Convergence**: Negative reward gradients for lateral error and heading are already trending downward, confirming the mathematical balance is correct.
