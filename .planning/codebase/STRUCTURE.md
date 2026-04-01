# Codebase Structure

**Analysis Date:** 2024-10-24

## Directory Layout

```
[project-root]/
├── arcproLab/          # Core Python logic and RL environment
│   ├── assets/         # Robot-specific USD files (F1Tenth_Metric.usd)
│   ├── mdp/            # Markov Decision Process implementation
│   │   ├── events.py   # Reset logic and spawning
│   │   ├── observations.py # 12-float telemetry and observation vectors
│   │   ├── rewards.py  # Reward functions (Gaussian weighted)
│   │   ├── terminations.py # Episode termination rules
│   │   └── track_manager.py # Absolute waypoint tracking and navigation math
│   ├── models/         # Trained model weights (.pth)
│   ├── policy_stack/   # Hierarchical and Fusion policy logic
│   └── scripts/        # Training and verification entry points
├── docs/               # Project documentation and roadmap
├── openStreetUSD/      # Map assets and world geometry
│   └── archive/        # Legacy/Experimental USD assets
├── tests/              # Pytest suite
├── trash/              # Legacy tools and one-off scripts
│   └── tools/          # USD manipulation and auditing tools
├── requirements.txt    # Python dependencies
└── train.sh            # Training execution script
```

## Directory Purposes

**arcproLab/mdp/:**
- Purpose: Logic for RL training iterations.
- Contains: Feature extraction (12-float protocol), reward shaping, and reset logic.
- Key files: `observations.py`, `rewards.py`, `track_manager.py`.

**arcproLab/policy_stack/:**
- Purpose: Advanced neural network architectures.
- Contains: Hierarchical policies and planning-control separation.
- Key files: `policies/hierarchical_policy.py`.

**trash/tools/:**
- Purpose: Infrastructure and asset preparation scripts.
- Contains: USD scaling (0.5x), centering, and auditing tools.
- Key files: `finalize_track_gsd.py`, `check_map_scale.py`.

## Key File Locations

**Entry Points:**
- `arcproLab/scripts/train_policy.py`: Main training entry point.
- `arcproLab/scripts/verify_policy.py`: Inference and verification entry point.

**Configuration:**
- `arcproLab/arcpro_env_cfg.py`: Environment and scene configuration (0.5x scale).
- `arcproLab/arcpro_robot_cfg.py`: Robot physical property configuration (FWD).

**Core Logic:**
- `arcproLab/mdp/track_manager.py`: Critical **absolute waypoint** navigation logic.

**Testing:**
- `tests/test_track_manager.py`: Logic tests for navigation math.

## Naming Conventions

**Files:**
- Snake case: `arcpro_env_cfg.py`.

**Directories:**
- CamelCase/SnakeCase: `arcproLab/`, `openStreetUSD/`.

## Where to Add New Code

**New Feature (MDP):**
- Primary code: `arcproLab/mdp/`.
- Tests: `tests/`.

**New Component/Module:**
- Implementation: `arcproLab/` or `arcproLab/policy_stack/`.

**Utilities:**
- Simulation-related: `arcproLab/mdp/`.
- Asset-related: `trash/tools/`.

## Special Directories

**openStreetUSD/archive/:**
- Purpose: Stores experimental map assets to prevent project root clutter.
- Generated: No.
- Committed: Yes.

**trash/:**
- Purpose: Legacy or "at-risk" scripts that are not part of the core training pipeline but provide necessary maintenance functions.
- Committed: Yes.

---

*Structure analysis: 2024-10-24*
