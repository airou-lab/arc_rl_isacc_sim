# Architecture

**Analysis Date:** 2026-07-28

## Pattern Overview

**Overall:** Reinforcement Learning (RL) with Isaac Sim & SKRL

**Key Characteristics:**
- Manager-based RL environment (`isaaclab.envs.ManagerBasedRLEnvCfg`)
- Custom PyTorch wrappers for SKRL models (`arcproLab.agents.skrl_wrappers`)
- Isaac Sim physics and rendering for simulation
- Custom observation, reward, and termination definitions in `mdp` layer

## Layers

**Environment Config Layer:**
- Purpose: Defines the environment, robot, and simulation settings
- Location: `arcproLab/arcpro_env_cfg.py`, `arcproLab/arcpro_robot_cfg.py`
- Contains: Environment classes and configuration variables
- Depends on: Isaac Lab `mdp` and core simulation utils
- Used by: Training and evaluation scripts

**MDP Layer (Observations, Rewards, Terminations, Actions):**
- Purpose: Core reinforcement learning mechanics definition
- Location: `arcproLab/mdp/`
- Contains: Action, observation, reward, and termination logic
- Depends on: PyTorch, Gymnasium, IsaacLab
- Used by: Environment Config Layer

**Agent/Policy Layer:**
- Purpose: Neural network models and RL wrappers
- Location: `arcproLab/agents/`, `arcproLab/policy_stack/`
- Contains: SKRL neural network definitions (ResNet18 base)
- Depends on: PyTorch, SKRL, Torchvision
- Used by: Training scripts

**Script/Execution Layer:**
- Purpose: Training loop, rendering, evaluation, and system administration
- Location: `arcproLab/scripts/`, `start_tmux_training.sh`
- Contains: Executable python scripts (e.g., `train_skrl.py`, `watchdog.py`)
- Depends on: All other layers, plus Isaac Lab launcher
- Used by: CLI user

## Data Flow

**Training Loop:**

1. `arcproLab/scripts/train_skrl.py` initializes the SKRL agent and `ARCProEnvCfg`
2. Environment passes image/telemetry via `mdp.observations`
3. SKRL PyTorch model computes steering/acceleration action
4. Environment applies action via `mdp.actions` and calculates step reward/terminations via `mdp.rewards` and `mdp.terminations`
5. Experience buffer is updated and PPO optimization runs

**State Management:**
- PPO memory buffer handled by `skrl.memories.RandomMemory`
- Environment state managed by Isaac Lab simulation contexts (`ManagerBasedEnv`)

## Key Abstractions

**Environment Config (`ARCProEnvCfg`):**
- Purpose: Configuration for IsaacLab environment
- Examples: `arcproLab/arcpro_env_cfg.py`
- Pattern: Configuration Class / Declarative setup

**MDP Terms (Rewards/Observations/Terminations):**
- Purpose: Mapping raw physics states into agent-consumable tensors
- Examples: `arcproLab/mdp/rewards.py`, `arcproLab/mdp/terminations.py`
- Pattern: Term Configuration

## Entry Points

**Training Script:**
- Location: `start_tmux_training.sh` -> `arcproLab/scripts/train_skrl.py`
- Triggers: User execution or cron restart
- Responsibilities: Initiating the RL training process in a tmux session, logging via TensorBoard

**Watchdog:**
- Location: `arcproLab/scripts/watchdog.py`
- Triggers: Background execution alongside training
- Responsibilities: Monitoring logs, auto-restarting on failure

**Track Generation:**
- Location: `arcproLab/generate_track.py`
- Triggers: User execution
- Responsibilities: Spawning USD environments

## Error Handling

**Strategy:** Fail-fast with watchdog recovery

**Patterns:**
- Watchdog script observes logs and restarts training automatically
- Standard PyTorch/Isaac exception handling in the training loops

## Cross-Cutting Concerns

**Logging:** Handled by TensorBoard (`logs/ppo_skrl`) and plain text output piped to `logs/skrl_phase1.log`
**Validation:** `arcproLab/scripts/` includes multiple `audit_` and `verify_` scripts (e.g. `audit_vision.py`, `verify_metric.py`)
**Configuration:** Centralized in Python config classes like `ARCProEnvCfg`

---

*Architecture analysis: 2026-07-28*
