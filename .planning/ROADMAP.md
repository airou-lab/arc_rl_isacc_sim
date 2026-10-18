# ARCPro RL Development Roadmap

**ClearSet Project Roadmap**  
**Last Updated:** 2026-10-18  
**Active Baseline:** `feature/realsense-realistic-kinematics` / `rollback/clean-baseline`

---

## Milestone M005: 48-Env Fleet Training & Full-Circuit Lap Mastery *(CURRENT)*
**Objective:** Achieve consistent, collision-free single-robot autonomous laps around the 50.72m simulated track using pure vision (RGB RealSense front-bumper mount, ResNet-18 + Spatial Softmax 2D keypoints).

- [x] **Phase 1: RealSense Kinematic Alignment & Physical Steering Unlock**
  - Raised revolute joint limits to $\pm 55.0^\circ$ and max steering angle to $0.68\text{ rad}$.
  - Grounded camera to physical front bumper mount `pos=(0.22, 0.0, 0.18)` at $15^\circ$ downward pitch.
- [x] **Phase 2: Vision Architecture Modernization (Levine et al. 2016 Spatial Softmax)**
  - Fused YUV linear preprocessing layer (`ITU-R BT.601`) for luminance (white lines) and chrominance (yellow median) separation.
  - Implemented `SpatialSoftmax2d` to extract 32 continuous $(x, y)$ keypoint coordinates, dropping actor parameters by 95% (453k $\to$ 21k).
  - Verified with deterministic gate test suite (`tests/test_vision_architecture_upgrade.py`).
- [/] **Phase 3: Autonomous Fleet Supervision & Sector 4 Intersection Convergence** *(IN PROGRESS)*
  - **Task 3.1:** 48-camera fleet training running in background under `autonomous_sentinel.py` supervisor, crossing 10M+ environment transitions.
  - **Task 3.2:** Dual-audit benchmarking against Golden Oracle (`compare_intersection_oracle.py`) and 5-sector regression suite (`checkpoint_auditor.py`).
  - **Target Exit Gate:** Full circuit traversal (>1,840 steps / 50.72m lap) at $\bar{v} \ge 0.50\text{ m/s}$ with zero boundary resets.

---

## Milestone M006: Smart Intersection Signals & Dynamic Traffic Coordination *(NEXT)*
**Objective:** Integrate active traffic light states, stop-bar compliance, and dynamic priority arbitration into the environment and policy.

- [ ] **Phase 1: Stop-Bar FSM & Visual Stop-Line Activation**
  - Connect `arcproLab/mdp/go_signal_manager.py` (3-state `APPROACH` $\to$ `STOP` $\to$ `DEPART` FSM) with `VisualStopLineDetector`.
  - Expose `go_signal` (0.0 / 1.0) into telemetry observation vector slot 1 and action throttle gating.
  - Enforce zero-velocity dwell at transverse stop bars until green go signal is issued.
- [ ] **Phase 2: Networked Intersection Node Server & Traffic Topology**
  - Integrate `arcproLab/policy_stack/agent/intersection_node_server.py` and `intersection_topology.json`.
  - Simulate multi-approach conflict arbitration (yield priority, round-robin green phases, pedestrian clearance intervals).
- [ ] **Phase 3: Closed-Loop Compliance Verification Gates**
  - Headless verification: Validate full stop at stop-bar line ($v = 0.0\text{ m/s}$), dwell for 3.0s, and smooth departure upon green go signal.

---

## Milestone M007: Multi-Robot Fleet Interaction & Forward Obstacle Detection / Avoidance *(UPCOMING)*
**Objective:** Introduce dynamic opponent vehicles into the simulation and equip the policy with low-latency obstacle avoidance and adaptive braking using RealSense depth.

- [ ] **Phase 1: 32-Bin Depth Corridor Slicing (1D Virtual Range Scan)**
  - Add `distance_to_image_plane` render buffer to `tiled_camera` in `ARCProSceneCfg`.
  - Implement zero-overhead horizontal min-pooling across chassis height rows (rows 90–130), downsampled into 32 angular distance bins ($\theta \in [-45^\circ, +45^\circ]$).
  - Expand `ARCProActor` control head input (`64 keypoints + 12 telemetry + 32 depth bins = 108 dims`).
- [ ] **Phase 2: Multi-Vehicle Scene Instantiation & Traffic Opponents**
  - Instantiate multi-vehicle environments in Isaac Sim (`{ENV_REGEX_NS}/LeadCar` or opponent policies).
  - Configure dynamic behaviors: slow cruising lead cars, stopped obstacles, and opposing traffic in intersection conflict zones.
- [ ] **Phase 3: Headway & Time-to-Collision (TTC) MDP Reward Shaping**
  - In `arcproLab/mdp/rewards.py`, add exponential proximity penalty for tailgating ($d_{\text{lead}} < 1.2\text{m}$):
    $$r_{\text{tailgate}} = -w_{\text{ttc}} \cdot \exp\left(-\frac{d_{\text{lead}}}{0.5}\right)$$
  - Add pre-braking incentive when $d_{\text{lead}} < 1.0\text{m}$.
  - Inter-chassis collision termination penalty ($-400.0$) when $d_{\text{inter-car}} < 0.35\text{m}$.
- [ ] **Phase 4: Sensor-Agnostic Abstraction & High-Speed Hardware Bridge**
  - Standardize the 32-bin range topic (`/arcpro/perception/range_scan_32`) for physical robot deployment.
  - Establish plug-and-play driver support for:
    1. RealSense D435i / D455 hardware stereo depth topic (`/camera/camera/depth/image_rect_raw`).
    2. Automotive mmWave radar (TI IWR6843) with direct Doppler velocity ($v_{\text{rel}}$) per bin.
    3. Dual MIPI CSI-2 global-shutter cameras (120–240 FPS, <2ms DMA latency directly to Jetson GPU).
