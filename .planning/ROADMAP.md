# ARCPro RL Development Roadmap

**ClearSet Project Roadmap**  
**Last Updated:** 2026-11-20  
**Active Baseline:** `rollback/clean-baseline` (canonical state tracked in `RESUME HERE.md` and `.cs/cs_turns.db`)

> **Strategic Rule:** Gate each major capability behind hard exit metrics so single-robot vision control, scene rendering (3DGS), generative weather augmentation (Cosmos), and multi-agent coordination (MARL) never compound debugging complexity simultaneously.

### Phased Master Plan Overview

| Phase | Milestone | Focus & Scope | Target Hardware / Stack | Exit Gate |
| :--- | :--- | :--- | :--- | :--- |
| **Phase 0** | `M005` | Single-robot RGB lap mastery on USD *(baking in background tmux `training` toward 500k–600k)* | RTX 3060 (12 GB) / Isaac Sim 5.1 | Full 50.72m lap (`>1,840 steps`, zero resets) |
| **Phase 1** | `M005.5` | Visual/collision proxy split, light/exposure DR, 1-env Cosmos/HDF5 recorder | RTX 3060 (12 GB) / Isaac Sim 5.1 | Default 48-env PPO unchanged; valid multimodal export |
| **Phase 2** | `M006`–`M007` | Multi-branch intersection gates *(Phase 01 Gates 1–3 DONE)*, stop-bar FSM (`GoSignalManager`), `RoadManager(N)` & opponent traffic TTC *(ACTIVE)* | RTX 3060 → Multi-GPU/Workstation | Multi-branch turn-token routing + compliant stop-bar dwell + collision-free traffic |
| **Phase 3** | `M008` | Camera-only Asymmetric MARL (`DirectMARLEnv` + `skrl` MAPPO) | RTX 3060 (small fleet) / >=48 GB GPU | Camera-only multi-car overtaking/yielding |
| **Phase 4** | `M009` | Real-world NuRec 3DGS `.usdz` track digital twin + RL fine-tuning | >24–48 GB GPU (e.g. RTX PRO 6000) + Isaac Sim 5.1 GA+/6.x | Full single & MARL laps in 3DGS scene |
| **Phase 5** | `M010` | Offline Cosmos Transfer weather/lighting augmentation + F1Tenth fleet sim-to-real | >=48–96 GB GPU / Cloud NIM | Zero-shot robustness across lighting/glare & real track |

---

## Milestone M005: 48-Env Fleet Training & Full-Circuit Lap Mastery *(Phase 0 — BACKGROUND BAKE)*
**Objective:** Achieve consistent, collision-free single-robot autonomous laps around the 50.72m simulated track using pure vision (RGB RealSense front-bumper mount, Equivariant ResNet-18 + Spatial Softmax 2D keypoints).

- [x] **Phase 1: RealSense Kinematic Alignment & Physical Steering Unlock**
  - Raised revolute joint limits to $\pm 55.0^\circ$ and max steering angle to $0.68\text{ rad}$.
  - Grounded camera to physical front bumper mount `pos=(0.22, 0.0, 0.18)` at $15^\circ$ downward pitch.
- [x] **Phase 2: Vision Architecture Modernization (Levine et al. 2016 Spatial Softmax + Equivariance)**
  - Implemented `SpatialSoftmax2d` + equivariant frontend $G(I)$ with parity policy head and $L_{\text{sym}}$, decoupled Actor/Critic optimizers, `asinh` odometer compression, and 2-slot ping-pong non-aliased wrapper buffers.
  - Verified with deterministic CPU invariant & architecture suites (`pytest -m cpu`).
- [/] **Phase 3: Autonomous Fleet Supervision & Baseline Baking to 500k–600k** *(IN PROGRESS)*
  - **Task 3.1:** 48-camera fleet training running in tmux `training` (resumed from `agent_450000.pt` after solving 48-env track overlap / `DistantLight` mismatch and six root-cause bug loops; see `RESUME HERE.md`).
  - **Task 3.2:** Dual-audit benchmarking against Golden Oracle (`compare_intersection_oracle.py`) and 5-sector regression suite (`checkpoint_auditor.py`).
  - **Target Exit Gate:** Full circuit traversal (>1,840 steps / 50.72m lap) at $\bar{v} \ge 0.50\text{ m/s}$ with zero boundary resets.

---

## Milestone M005.5: Scene Decoupling, Visual DR & Cosmos Capture Scaffolding *(Phase 1 — PREP)*
**Objective:** Build zero-overhead abstractions for future NuRec 3DGS scenes, Replicator visual domain randomization, and single-env multi-modal rollout recording without disturbing default 48-env PPO training.

- [ ] **Track Backend & Collision Decoupling**
  - Parameterize `ARCProSceneCfg.track` (`--track_backend {usd,nurec}` + `--track_usd_path`) to isolate the invisible PhysX collision proxy & `TrackManager` / `RoadManager` geometry frame from swappable visual assets.
- [ ] **Lightweight Visual Domain Randomization (RTX 3060-Safe)**
  - Add gated `DistantLightCfg` intensity/angle/color and camera exposure/noise randomization toggles (`isaacsim.replicator.domain_randomization` / event terms) for post-convergence fine-tuning.
- [ ] **Single-Env Multimodal Rollout Recorder**
  - Add `--record_cosmos` / `--record_hdf5` mode to `play_skrl.py` (`--num_envs 1`) capturing synchronized `rgb`, `semantic_segmentation`, `distance_to_image_plane`, `normals`, and `(obs, action)` logs alongside ARCPro prompt templates compatible with `IsaacLab/scripts/tools/cosmos/cosmos_prompt_gen.py`.

---

## Milestone M006: Smart Intersection Signals, Multi-Branch Topology & Dynamic Traffic Coordination *(ACTIVE)*
**Objective:** Enable command-conditioned multi-branch navigation (`turn_token in {-1.0, 0.0, +1.0}`) across the 3-loop USD track (`junction_55`, `junction_18`, C2) and integrate active traffic light states, stop-bar compliance, and dynamic priority arbitration.

- [x] **Phase 01: Multi-Branch Intersection Gates (Gates 1, 2 & 3 — Spec: `.planning/phases/M006-01-SPEC.md`)**
  - **Gate 1 (`track_manager.py`):** Pre-materialized 3-branch `(3, 5000, 3)` waypoint & curvature tensor bank (`Left/-1.0`, `Straight/0.0`, `Right/+1.0`) with bit-exact M005 single-loop default (`randomize_turn_tokens=False`).
  - **Gate 2 (`track_topology.py`, `terminations.py`):** Token-gated spatial permeability (`in_branch_permeable_gate`, token-aware `in_off_cliff_east` & `in_wrong_lane_intersection`) without modifying `track_boundaries_1x.npz`.
  - **Gate 3 (`road_manager.py`, `rewards.py`, `events.py`):** Vectorized `RoadManager` mission-token FSM ordered before `TrackManager.compute_errors()`, curvature-sign-symmetric `lateral_error_reward`, and branch-switch re-anchoring in `waypoint_progress_reward`. Verified (`44/44` in `tests/test_multi_branch_gates.py`).
- [/] **Phase 01b: Stage 1/2/3 Curriculum Turn-Token Activation** *(QUEUED ON M005 500k–600k BAKE)*
  - Commit Gates 1–3 upon approval, then activate `ARCPRO_RANDOMIZE_TURN_TOKENS=1` / `rm.set_randomization(True)` warm-started from the 500k+ M005 baseline.
- [ ] **Phase 02: Stop-Bar FSM & Visual Stop-Line Activation**
  - Connect `arcproLab/mdp/go_signal_manager.py` (3-state `APPROACH` $\to$ `STOP` $\to$ `DEPART` FSM) with `VisualStopLineDetector`.
  - Expose `go_signal` (0.0 / 1.0) into telemetry observation vector slot 1 and action throttle gating.
  - Enforce zero-velocity dwell at transverse stop bars until green go signal is issued.
- [ ] **Phase 03: Networked Intersection Node Server & Closed-Loop Compliance Gates**
  - Integrate `arcproLab/policy_stack/agent/intersection_node_server.py` and `intersection_topology.json`.
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

---

## Milestone M008: Camera-Only Asymmetric MARL *(Phase 3 — RESEARCH CORE)*
**Objective:** Train decentralized, camera-only multi-agent policies using Asymmetric Actor-Critic MARL (`DirectMARLEnv` + `skrl` MAPPO) so fleets coordinate lane-keeping, yielding, merging, and overtaking without V2V/GPS leakage at deployment.

- [ ] **Phase 1: Multi-Agent Environment & `RoadManager(N)` Generalization**
  - Generalize `RoadManager`, `GoSignalManager`, and curriculum spawners (`mdp/events.py`) from single-robot (`N=1`) to `N` concurrent ego agents with non-overlapping waypoint spawns.
  - Wrap task on Isaac Lab `DirectMARLEnv` / `DirectMARLEnvCfg` compatible with `IsaacLab/scripts/reinforcement_learning/skrl/train.py` (`IPPO` / `MAPPO`).
- [ ] **Phase 2: Asymmetric Observation & Centralized Critic**
  - **Actor (Decentralized, Camera-Only):** Pure front RGB (`SpatialSoftmax2d` + short temporal/action history + discrete `turn_token`/`go_signal`) — no privileged state or neighbor coordinates.
  - **Critic (Centralized, Privileged MARL State):** Global multi-agent poses, relative velocities, TTC matrix, and intersection queue order in `ARCProCritic`.
- [ ] **Phase 3: Interactive Fleet Verification Gate**
  - Headless & headed verification: Zero inter-car collisions across multi-car overtaking, yielding, and 4-way intersection conflict scenarios from RGB-only actors.

---

## Milestone M009: Real-World 3DGS Track Twin via NVIDIA Omniverse NuRec *(Phase 4 — REAL-TO-SIM)*
**Objective:** Reconstruct the physical test track into a photorealistic 3D Gaussian Splatting (`.usdz`) digital twin and fine-tune converged USD policies inside it. *(Requires >24–48 GB GPU + Isaac Sim 5.1 GA+ / 6.x `omni.usd.schema.omni_nurec_types` / `isaacsim.replicator.nurec_utils`.)*

- [ ] **Phase 1: Real-World Track Capture & 3DGRUT Reconstruction**
  - Record multi-view track video -> reconstruct 3DGS checkpoint via `nv-tlabs/3dgrut` / NuRec pipeline -> export `.usdz` (`OmniNuRecVolume` / `ParticleField`).
- [ ] **Phase 2: Hybrid Collision Proxy Alignment & Pre-Hydra Setup**
  - Mount the reconstructed `.usdz` into `M005.5`'s visual track slot aligned to the metric PhysX collision proxy + `TrackManager` centerline/gates; verify clean RGB-only rendering (disabling unsupported splat annotators/frame-gen artifacts).
- [ ] **Phase 3: KL/BC-Anchored RL Fine-Tuning in 3DGS**
  - Warm-start from converged Phase 0/3 checkpoints and fine-tune inside 3DGS with policy regularization (RAD/OPTED pattern) to close visual appearance gaps without forgetting kinematics.

---

## Milestone M010: Cosmos Transfer Hardening & Physical F1Tenth Deployment *(Phase 5 — SIM-TO-REAL)*
**Objective:** Multiply rollout diversity across unseen weather, shadows, glare, and surface textures via NVIDIA Cosmos Transfer and deploy onto the physical F1Tenth / Jetson fleet. *(Requires >=48–96 GB GPU or cloud NIM for Cosmos Transfer inference.)*

- [ ] **Phase 1: Multi-Modal Capture & Prompt Suite Generation**
  - Run `M005.5` rollout capture (`CosmosWriter` or Isaac Lab HDF5 -> `hdf5_to_mp4.py`) on converged policies; generate domain-coherent ARCPro track prompts via `cosmos_prompt_gen.py`.
- [ ] **Phase 2: Offline Cosmos Transfer Augmentation & Encoder Distillation**
  - Generate lighting/weather/glare/asphalt variations (`rgb`, `depth`, `edges`, `shaded_seg` Multi-ControlNet) -> repackage via `mp4_to_hdf5.py` & `merge_hdf5_datasets.py` -> distill/fine-tune visual features.
- [ ] **Phase 3: Physical F1Tenth Fleet Sim-to-Real Validation**
  - Deploy quantized camera-only policy (`SpatialSoftmax2d` + RealSense D455) onto physical F1Tenth chassis; benchmark single-car lap consistency and multi-car visual coordination on track.

