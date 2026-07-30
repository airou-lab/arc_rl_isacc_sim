# Codebase Structure

**Analysis Date:** 2026-07-28

## Directory Layout

```
/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/
├── arcproLab/          # Core environment and RL implementation
├── .planning/          # AI planning and history files
├── logs/               # Log output, Tensorboard events, and PyTorch checkpoints
├── openStreetUSD/      # Environment/track USD assets
├── archive/            # Old logs and legacy runs
├── debug_frames/       # Output for visual debugging scripts
└── [root scripts]      # High-level entry points and shell scripts
```

## Directory Purposes

**`arcproLab/`:**
- Purpose: Main Python module for RL and simulation logic
- Contains: PyTorch models, MDP logic, environment configs, and training scripts
- Key files: `arcpro_env_cfg.py`, `arcpro_robot_cfg.py`

**`arcproLab/mdp/`:**
- Purpose: Markov Decision Process (MDP) terms for reinforcement learning
- Contains: Python files defining actions, observations, rewards, and terminations
- Key files: `actions.py`, `rewards.py`, `terminations.py`, `observations.py`

**`arcproLab/scripts/`:**
- Purpose: Execution layer for training, auditing, and playing
- Contains: PyTorch training scripts and verification tools
- Key files: `train_skrl.py`, `watchdog.py`, `play_skrl.py`

**`arcproLab/agents/`:**
- Purpose: SKRL neural network definitions and wrappers
- Contains: Model classes inheriting from PyTorch/SKRL
- Key files: `skrl_models.py`, `skrl_wrappers.py`

**`logs/`:**
- Purpose: Storage for run output and model checkpoints
- Contains: `.log` files and `ppo_skrl/` TensorBoard folders
- Key files: `skrl_phase1.log`

## Key File Locations

**Entry Points:**
- `start_tmux_training.sh`: Launches the entire training pipeline in background
- `arcproLab/scripts/train_skrl.py`: Main python RL training loop
- `arcproLab/scripts/play_skrl.py`: Agent execution/inference visualizer

**Configuration:**
- `arcproLab/arcpro_env_cfg.py`: Environment setup and reward weights
- `arcproLab/arcpro_robot_cfg.py`: Robot physical characteristics

**Core Logic:**
- `arcproLab/mdp/`: All the reward, termination, and observation math

**Testing:**
- `arcproLab/scripts/test_*.py`: Isolated tests for physics, rendering, and logic
- Root `test_*.py` files: Ad-hoc python tests

## Naming Conventions

**Files:**
- Snake Case: `train_skrl.py`, `arcpro_env_cfg.py`

**Directories:**
- Mixed (mostly snake case, some camel/pascal): `arcproLab`, `policy_stack`, `openStreetUSD`

## Where to Add New Code

**New Feature:**
- Primary code: Add into `arcproLab/mdp/` or `arcproLab/policy_stack/` depending on scope
- Tests: Add a corresponding `test_*.py` to root or `arcproLab/scripts/`

**New Component/Module:**
- Implementation: Inside `arcproLab/`

**Utilities:**
- Shared helpers: Inside `arcproLab/mdp/` or root

## Special Directories

**`.planning/`:**
- Purpose: Storage for agent-specific context, prompts, and history (e.g. `monitor_agent_prompt.md`, `RESUME.md`)
- Generated: Yes (by agents)
- Committed: Yes

**`logs/`:**
- Purpose: Active training logs and models
- Generated: Yes (by Isaac Sim / SKRL)
- Committed: No (usually gitignored)

---

*Structure analysis: 2026-07-28*
