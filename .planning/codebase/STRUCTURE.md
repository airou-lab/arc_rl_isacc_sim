# Codebase Structure

**Analysis Date:** 2024-05-14

## Directory Layout

```
[project-root]/
├── arcproLab/          # Core RL environment package
│   ├── arcpro_env_cfg.py # Main environment configuration
│   ├── generate_track.py # Utility to generate waypoints from USD
│   └── mdp/            # Markov Decision Process components
│       ├── events.py       # Simulation events (startup, reset)
│       ├── observations.py # Sensor to observation vector logic
│       ├── rewards.py      # Reward signal calculation logic
│       ├── track_manager.py # Logic for track handling and errors
│       └── track_centerline.npy # Persisted track waypoint data
├── openStreetUSD/      # Isaac Sim / Omniverse scene files
│   ├── arcpro_RL_open_street_sim.usd # Main environment USD
│   └── no_graph_sim.usd # Backup scene
├── tests/              # Unit and integration tests
│   └── test_track_manager.py # Tests for TrackManager logic
└── requirements.txt    # Python dependencies
```

## Directory Purposes

**arcproLab/:**
- Purpose: Contains all logic and configuration for the ARCPro RL environment using Isaac Lab.
- Contains: Configuration classes and environment utility scripts.
- Key files: `arcpro_env_cfg.py`, `generate_track.py`

**arcproLab/mdp/:**
- Purpose: Implements the "Manager-Based" terms required by Isaac Lab.
- Contains: Logic for observations, rewards, events, and terminations.
- Key files: `track_manager.py`, `observations.py`, `rewards.py`

**openStreetUSD/:**
- Purpose: Holds the source of truth for the physical environment.
- Contains: USD (Universal Scene Description) files containing road geometry, textures, and lighting.
- Key files: `arcpro_RL_open_street_sim.usd`

## Key File Locations

**Entry Points:**
- `arcproLab/arcpro_env_cfg.py`: The "recipe" for the environment used by training scripts.
- `arcproLab/generate_track.py`: Tool for preprocessing USD road data into waypoints.

**Configuration:**
- `arcproLab/arcpro_env_cfg.py`: Central configuration class `ARCProEnvCfg`.

**Core Logic:**
- `arcproLab/mdp/track_manager.py`: Core mathematical logic for lateral/heading error calculation.
- `arcproLab/mdp/observations.py`: Defines the 12-element telemetry vector.

**Testing:**
- `tests/test_track_manager.py`: Validates the `TrackManager` logic (loading, errors, sampling).

## Naming Conventions

**Files:**
- `*_cfg.py`: Isaac Lab configuration files (uses `@configclass`).
- `*.usd`: Universal Scene Description files.
- `*.npy`: NumPy data files (used for persisted waypoints).

**Classes:**
- `*Cfg`: Configuration classes (e.g., `ARCProSceneCfg`).
- `*Manager`: Singleton logic managers (e.g., `TrackManager`).

## Where to Add New Code

**New Reward Signal:**
- Add a function to `arcproLab/mdp/rewards.py`.
- Register it in the `RewardCfg` class within `arcproLab/arcpro_env_cfg.py`.

**New Observation (e.g., LiDAR):**
- Add the calculation logic to `arcproLab/mdp/observations.py`.
- Define it in `ObservationCfg` in `arcproLab/arcpro_env_cfg.py`.

**New Simulation Environment (Map):**
- Place the USD file in `openStreetUSD/`.
- Update `track.usd_path` in `ARCProSceneCfg` (in `arcproLab/arcpro_env_cfg.py`).
- Re-run `arcproLab/generate_track.py` to update waypoints.

## Special Directories

**arcproLab/mdp/track_centerline.npy:**
- Purpose: Cached waypoints extracted from the USD.
- Generated: Yes (by `generate_track.py`).
- Committed: Yes (essential for runtime without sampling overhead).

---

*Structure analysis: 2024-05-14*
