# Codebase Structure

**Analysis Date: 2026-05-11**

## Directory Layout

```
arc_rl_isacc_sim/
├── arcproLab/              # Main source code
│   ├── assets/             # Robot (F1Tenth_Metric) and environment assets
│   ├── mdp/                # MDP components (observations, rewards, terminations)
│   ├── models/             # Saved policy models
│   ├── policy_stack/       # RL implementations (Submodule)
│   └── scripts/            # Operational scripts (train, verify, generate track)
├── debug_frames/           # Captured frames for visual debugging (Untracked)
├── docs/                   # High-level documentation
├── logs/                   # Training logs and TensorBoard data
├── openStreetUSD/          # 3D assets and scene files for Isaac Sim
├── tensorboard_view_docker/# Docker configuration for viewing TensorBoard
├── trash/                  # Deprecated or backup files (Untracked)
└── .planning/              # Project planning and phase documentation
    └── phases/
        ├── 11-intersection-navigation/
        ├── 12-autonomous-navigation/
        ├── 13-live-policy-gui/
        ├── 14-01-procedural-multi-agent-scaffolding/
        └── 15-hd-vision-resnet/ # Current active phase
```

## Directory Purposes

**arcproLab/mdp/:**
- Purpose: Defines the MDP interface and simulation logic.
- Key files: `track_manager.py`, `road_graph.py`, `events.py`, `terminations.py`, `track_boundaries_1x.npz`, `spawner.py`, `debug_terminations.py`.

**arcproLab/policy_stack/:**
- Purpose: RL framework and Isaac Lab environment wrappers (Submodule).
- Key files:
  - `policies/hierarchical_policy.py`: Recurrent hierarchical policy with waypoint planning.
  - `policies/fusion_policy.py`: Flexible HD Multi-Modal perception (Adaptive CNN + Telemetry).
  - `wrappers/waypoint_tracking_wrapper.py`: Context wrapper for hierarchical objectives.

**openStreetUSD/:**
- Purpose: 3D assets for Isaac Sim using USD format.
- Key files: `no_graph_sim_clean_1x_flattened.usda`, `track_1x_wrapper.usda`.

**logs/:**
- Purpose: Stores training progress and telemetry.
- Key files: `production_hd_resnet_v14.log` (Simulated reference for v14 hardening).

---

*Structure analysis: 2026-05-11*
