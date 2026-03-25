# Isaac Lab Scaling Research

## 1. Map Proportion Analysis
**Target**: `no_graph_sim.usd`
**Measured Road Lane**: ~42.4 meters.
**F1Tenth Width (Raw)**: ~0.25 meters.
**Alignment Factor**: ~170x scale needed.

## 2. Scaling Stability (PhysX)
- **1.0x Scale**: (Stable) Robot is too small for the map.
- **10.0x Scale**: (Stable) Fits better but still small.
- **20.0x Scale**: (Stable) Robot is clearly visible, house-sized.
- **170.0x Scale**: (CRITICAL FAILURE) PhysX engine crashes during initialization due to extreme mass/inertia values.

## 3. Contact Solver Tuning
- **Vibration Problem**: Robot gets stuck in place while wheels spin/vibrate.
- **Collision Sandwich**: Occurs when both GroundPlane ($Z=0$) and Track ($Z=0.01$) are active.
- **Current Fix**: Remove GroundPlane, use 5cm `contact_offset` to handle giant mesh collision resolution.
