# Codebase Structure

**Analysis Date:** 2026-05-11

## Directory Layout

```
arcproLab/
├── assets/                 # Robot and environment USD assets
├── mdp/                    # MDP logic (observations, rewards, terminations)
│   ├── road_manager.py     # Multi-agent navigation state management
│   └── track_manager.py    # Track geometry and boundary management
├── policy_stack/           # RL Framework
│   ├── agent/              # MARL arbitration and coordination logic
│   ├── policies/           # Neural network architectures
│   ├── tests/              # Unit and integration tests
│   └── wrappers/           # Environment wrappers for RL
├── scripts/                # Utility and entry-point scripts
└── models/                 # Trained model checkpoints
openStreetUSD/              # USD scene files and track definitions
.planning/                  # Project roadmap and codebase maps
```

## Directory Purposes

**arcproLab/mdp/:**
- Purpose: Markov Decision Process components for Isaac Lab.
- Contains: Logic for calculating observations, rewards, and environment reset events.
- Key files: `road_manager.py`, `track_manager.py`, `rewards.py`, `observations.py`.

**arcproLab/policy_stack/agent/:**
- Purpose: Multi-agent coordination and arbitration.
- Contains: FCFS scheduler, intersection graph logic, and network transport facades.
- Key files: `scheduler_core.py`, `worker_scheduler.py`, `intersection_node_server.py`.

**arcproLab/policy_stack/policies/:**
- Purpose: Policy network implementations.
- Key files: `hierarchical_policy.py` (Waypoint planning), `fusion_policy.py` (Perception).

**arcproLab/scripts/:**
- Purpose: Operational scripts for the project.
- Key files: `train_policy.py`, `verify_policy.py`, `generate_track.py`.

## Key File Locations

**Entry Points:**
- `arcproLab/scripts/train_policy.py`: Primary training script.
- `arcproLab/policy_stack/train_policy_ros2.py`: ROS2-compatible training script.

**Configuration:**
- `arcproLab/arcpro_env_cfg.py`: Environment and scene configuration.
- `arcproLab/arcpro_robot_cfg.py`: Robot sensor and actuator configuration.

**Core Logic:**
- `arcproLab/mdp/road_manager.py`: Navigation command management.
- `arcproLab/policy_stack/agent/scheduler_core.py`: Arbitration logic.

**Testing:**
- `arcproLab/policy_stack/tests/`: Unit tests for arbitration and RL components.

## Naming Conventions

**Files:**
- snake_case: `road_manager.py`, `hierarchical_policy.py`.

**Directories:**
- snake_case: `policy_stack`, `debug_frames`.

## Where to Add New Code

**New MDP Term (Reward/Observation):**
- Primary code: `arcproLab/mdp/` in the respective file (`rewards.py`, etc.).
- Config: `arcproLab/arcpro_env_cfg.py`.

**New Policy Feature:**
- Implementation: `arcproLab/policy_stack/policies/`.

**New Multi-Agent Coordination Logic:**
- Implementation: `arcproLab/policy_stack/agent/scheduler_core.py`.

## Special Directories

**trash/:**
- Purpose: Contains deprecated files like `road_graph.py.bak`.
- Generated: No.
- Committed: No (ignored by .gitignore).

---

*Structure analysis: 2026-05-11*
