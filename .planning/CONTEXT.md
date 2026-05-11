# Project Context: ARCPro RL v1.2-dev

## Environment State
- **Scale**: 1.0x Metric (True Physics).
- **Mode**: "Road in Void" (Grass/Fences ghosted).
- **Orientation**: **South-facing** (-1.57 rad).
- **Target**: Right Lane Center (+2.42m from Yellow Line).
- **Boundaries**: Strict Reset (Safe Zone: 0.225m to 2.5m).

## Control & Physics
- **Frequency**: 500Hz (`dt=0.002`).
- **Control Rate**: 20Hz (`decimation=25`).
- **Robot**: 20kg mass, 5Nm torque limits.
- **Action Fix**: **Inverted Throttle** (Scale: -60.0) to fix backwards driving.

## Policy Configuration
- **Vision-only**: `lat_err` and `head_err` are **masked (0.0)** in the policy observation vector to force reliance on the RGB camera.
- **Rewards**: Logic uses unmasked `env.extras["lat_err"]` for ground-truth feedback.
- **Camera**: Mimics **Intel RealSense D435i Wide** (90° HFOV), tilted 10° down.

## Next Step
- **Retrain Phase 11**: Launch 32-env production run (`bash train.sh --num_envs 32 --headless`).
