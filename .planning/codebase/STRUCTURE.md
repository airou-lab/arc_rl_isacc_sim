# Codebase Structure

**Analysis Date:** 2024-05-24

## Directory Layout

```
arc_rl_isacc_sim/
├── arcproLab/                 # Core Autonomous Driving Simulation & Policy Package
│   ├── assets/                # 3D geometries, textures, and USD scenes
│   ├── mdp/                   # Markov Decision Process Term Implementations
│   ├── models/                # Checkpoints and saved weights for trained models
│   ├── policy_stack/          # Neural Architectures and Hierarchical Agent Logic
│   └── scripts/               # Executable entry points for training, testing, and utilities
├── openStreetUSD/             # OpenStreetMap to USD conversion and world generation tools
└── logs/                      # Output logs, Tensorboard events, and execution runs
```

## Directory Purposes

**`arcproLab/mdp/`:**
- Purpose: Defines the rules of the reinforcement learning problem.
- Contains: Standalone Python files handling observations, actions, rewards, terminations, and events.
- Key files: `observations.py`, `rewards.py`, `actions.py`, `terminations.py`

**`arcproLab/policy_stack/agent/`:**
- Purpose: Implements the non-learned, classical logic of the autonomous agent.
- Contains: Hierarchical node structures, topological routing, and scheduler logic.
- Key files: `agent_node.py`, `worker_scheduler.py`, `intersection_graph.py`

**`arcproLab/policy_stack/policies/`:**
- Purpose: Implements the neural network architectures used by the Stable-Baselines3 agents.
- Contains: Custom PyTorch neural network classes.
- Key files: `fusion_policy.py`, `hierarchical_policy.py`

**`arcproLab/scripts/`:**
- Purpose: The primary execution surface for users and CI pipelines.
- Contains: Entry points to trigger simulation, training, policy deployment, or to verify aspects of the environment.
- Key files: `train_policy.py`, `verify_policy.py`, `audit_live.py`

## Key File Locations

**Entry Points:**
- `arcproLab/scripts/train_policy.py`: The main script to bootstrap Isaac Lab, wrap it for Stable-Baselines3, and begin training.

**Configuration:**
- `arcproLab/arcpro_env_cfg.py`: The central declarative configuration that maps Isaac Sim parameters, rendering targets, and physics specifics to the RL Environment.
- `arcproLab/arcpro_robot_cfg.py`: Configuration specifically defining the F1Tenth vehicle's joints, articulation, and initial states.

**Core Logic:**
- `arcproLab/policy_stack/agent/agent_node.py`: Holds the main `AgentNode` mapping high-level "where to go" routing to neural "how to get there" logic.

**Testing:**
- `arcproLab/scripts/test_env.py` and `arcproLab/scripts/test_sb3_wrapper.py`: Validation scripts ensuring the environment steps correctly and interfaces cleanly with standard RL abstractions.

## Naming Conventions

**Files:**
- lowercase_with_underscores: Standard PEP8 module naming is rigorously used (e.g., `train_policy.py`, `fusion_policy.py`).

**Directories:**
- lowercase_with_underscores: `policy_stack`, `models`, `scripts`. The exception is the root package `arcproLab` and the sub-package `openStreetUSD`.

## Where to Add New Code

**New Feature (RL Term):**
- Primary code: Define a new function in the relevant `arcproLab/mdp/` file (e.g., `rewards.py`).
- Integration: Instantiate the term inside a `@configclass` block within `arcproLab/arcpro_env_cfg.py`.

**New Neural Architecture:**
- Implementation: Add the new network design to `arcproLab/policy_stack/policies/`.

**Utilities:**
- Shared helpers: Add non-execution utilities to specific sub-folders like `arcproLab/policy_stack/agent/` if they are autonomous driving tools, or `arcproLab/mdp/` if they manipulate states.

## Special Directories

**`logs/`:**
- Purpose: Contains output logs from training runs, TensorBoard summaries, and wandb sync data.
- Generated: Yes
- Committed: No (usually ignored in version control).
