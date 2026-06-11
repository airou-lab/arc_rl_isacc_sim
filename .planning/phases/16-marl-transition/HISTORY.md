# Phase 16: MARL Transition - History

## Physics Stabilization: The "Control Flip" Strategy
During the transition to MARL, we encountered severe physics instability and NaN crashes. The root cause was forcing the USD asset upright, which broke the PhysX suspension.

### Bugs Encountered
- **PhysX Suspension Breakage**: Applying a WXYZ orientation of `(0.0, 0.7071, 0.7071, 0.0)` in `arcpro_env_cfg.py` to force the robot upright caused the suspension system to behave erratically, leading to NaN values and simulation crashes.
- **Stagnation Termination Sensitivity**: The 10-step stagnation limit was too strict for an uninitialized RL agent trying to start from a dead stop, causing immediate episode truncation. Also, local axes flips caused velocity tracking issues.

### Fixes Applied
1. **Reverted Orientation (`events.py`)**: Restored the native USD upside-down spawn orientation. This keeps the PhysX articulation stable.
   ```python
   # Explicitly build WXYZ quaternion for 1.0x native orientation
   quats[i, 0] = math.cos(half_yaw) # W
   quats[i, 1] = 0.0                # X
   quats[i, 2] = 0.0                # Y
   quats[i, 3] = math.sin(half_yaw) # Z
   ```

2. **Control Flip (`arcpro_env_cfg.py`)**: To compensate for the upside-down chassis, we flipped the drive control polarity.
   ```python
   drive = arcpro_actions.CombinedDriveActionCfg(
       asset_name="robot",
       joint_names=["Joint_Drive_RL", "Joint_Drive_RR", "Joint_Drive_FL", "Joint_Drive_FR"],
       scale=-20.0, # Flipped polarity
       offset=0.0
   )
   ```

3. **Stagnation Termination Update (`terminations.py`)**: Updated to use absolute velocity in the XY plane and increased the step limit to 50.
   ```python
   vel = torch.norm(asset.data.root_lin_vel_b[:, :2], dim=1)
   # Reset if speed is less than 0.1 m/s for more than 50 steps (~1.0s)
   return (vel < 0.1) & (env.episode_length_buf > 50)
   ```

### Convergence Metrics (Post-Fix)
The fresh training run is highly stable. At ~300k steps:
- **ep_len_mean**: ~40 steps (climbing steadily from ~8).
- **std**: Decayed from 1.01 to 0.66.
- **explained_variance**: Solidly positive (0.33).
The network successfully learns to drive and avoid walls.
