# Architecture

**Analysis Date:** 2026-03-26

## Pattern Overview

**Overall:** Layered Architecture for Reinforcement Learning (RL) within an Omniverse Isaac Lab Simulation Environment.

**Key Characteristics:**
- **Simulation-centric:** Built on NVIDIA Isaac Lab for robotics simulation.
- **Modular MDP Definition:** Clear separation of observation, reward, termination, and event logic for RL environments.
- **Configuration-driven:** Environment and robot parameters are managed through dedicated configuration files.
- **Policy-agnostic RL Training:** Training scripts interact with the defined environment to train various RL policies.

## Layers

**Simulation Layer:**
- Purpose: Provides the physics simulation, rendering, and core environment functionalities using NVIDIA Isaac Lab.
- Location: Integrated via `isaaclab.sim` and `isaaclab.envs.mdp` imports.
- Contains: Low-level simulation primitives, physics engine, scene graph management.
- Depends on: Omniverse Platform, underlying physics engine.
- Used by: All RL environment definitions and training scripts.

**RL Environment Definition Layer (MDP):**
- Purpose: Defines the Markov Decision Process (MDP) for specific robotic tasks, including states (observations), actions, rewards, and episode termination conditions.
- Location: `arcproLab/mdp/`
- Contains:
    - `observations.py`: Logic for computing agent observations.
    - `rewards.py`: Logic for computing reward signals.
    - `terminations.py`: Logic for determining episode termination.
    - `events.py`: Logic for handling specific environmental events.
    - `policy_wrapper.py`: Adapts policies for use within the environment.
    - `track_manager.py`: Manages track generation and data.
- Depends on: `torch`, `numpy`, Simulation Layer (implicitly through Isaac Lab).
- Used by: Configuration Layer, Training/Verification Layer.

**Configuration Layer:**
- Purpose: Configures the Isaac Lab environment and robot assets based on the defined MDP components.
- Location: `arcproLab/arcpro_env_cfg.py`, `arcproLab/arcpro_metric_env_cfg.py`, `arcproLab/arcpro_robot_cfg.py`, `arcproLab/arcpro_metric_robot_cfg.py`
- Contains: Environment settings, robot properties, physics parameters.
- Depends on: RL Environment Definition Layer (MDP), Simulation Layer.
- Used by: Training/Verification Layer.

**Training/Verification Layer:**
- Purpose: Contains the executable scripts for running reinforcement learning training loops, evaluating trained policies, and verifying simulation setups.
- Location: `arcproLab/scripts/`
- Contains: `train_policy.py`, `verify_policy.py`, `verify_metric.py`, `verify_spawn.py`
- Depends on: Configuration Layer, RL Environment Definition Layer (MDP), Simulation Layer.
- Used by: Developers and CI/CD pipelines to run experiments.

**Asset Layer:**
- Purpose: Stores 3D models (USD files) and other static assets required for the simulation environment.
- Location: `arcproLab/assets/robot/`, `openStreetUSD/`
- Contains: Robot models, track geometries, environment scene descriptions.
- Depends on: None.
- Used by: Simulation Layer (loaded by Isaac Lab).

## Data Flow

**RL Training Flow:**

1.  **Script Execution:** `train.sh` (or a similar script) invokes `arcproLab/scripts/train_policy.py` via `isaaclab.sh`.
2.  **Environment Initialization:** The training script uses configuration from `arcproLab/*_env_cfg.py` and `arcproLab/*_robot_cfg.py` to initialize an Isaac Lab RL environment.
3.  **MDP Integration:** The initialized environment integrates observation, reward, termination, and event logic defined in `arcproLab/mdp/`.
4.  **Policy Interaction:** The RL agent interacts with the environment:
    *   Agent takes an action.
    *   Environment executes action in simulation, computes next state, reward (from `mdp/rewards.py`), and checks for termination (from `mdp/terminations.py`).
    *   Environment returns observation (from `mdp/observations.py`), reward, and done signal to the agent.
5.  **Policy Update:** The agent uses collected data to update its policy parameters (e.g., in `arcproLab/models/road_following_model.pth`).

**State Management:**
- Environment state (robot pose, joint states, physics objects) is managed by the Isaac Lab simulation core.
- RL-specific state (e.g., episode progress, cumulative reward) is managed within the RL environment wrapper and the training algorithm.
- Policy parameters are managed by PyTorch.

## Key Abstractions

**RL Environment:**
- Purpose: Standardizes the interface between an RL agent and the simulation, typically following the Gymnasium API.
- Examples: Implied by how Isaac Lab environments are constructed from the `arcproLab/*_env_cfg.py` files.
- Pattern: Observation, Action, Reward, Done, Info (OARDI) loop.

**MDP Components:**
- Purpose: Encapsulate individual aspects of the Markov Decision Process (Observations, Rewards, Terminations, Events).
- Examples: `arcproLab/mdp/observations.py`, `arcproLab/mdp/rewards.py`
- Pattern: Functions/classes that compute specific outputs based on simulation state.

## Entry Points

**Training Script:**
- Location: `arcproLab/scripts/train_policy.py`
- Triggers: Invoked by shell scripts like `train.sh` via `isaaclab.sh`.
- Responsibilities: Sets up the RL trainer, initializes the environment, manages the training loop, saves policies.

**Verification Scripts:**
- Location: `arcproLab/scripts/verify_policy.py`, `arcproLab/scripts/verify_metric.py`, `arcproLab/scripts/verify_spawn.py`
- Triggers: Invoked directly or via shell scripts (e.g., `verify_sim.sh`).
- Responsibilities: Loads a trained policy, runs it in the simulation, collects data, and evaluates performance; or verifies specific environment aspects.

## Error Handling

**Strategy:** Python's exception handling for application logic, with Isaac Lab handling simulation-specific errors internally (often resulting in environment resets or crashes).

**Patterns:**
- Command-line argument parsing often includes error checks.
- File I/O operations (e.g., loading models or track data) likely include `try-except` blocks.

## Cross-Cutting Concerns

**Logging:** Standard Python `logging` module is likely used, or `print` statements during development/debugging. Isaac Lab also provides its own logging.
**Validation:** Input arguments for scripts, configuration parameters.
**Authentication:** Not directly applicable to the RL simulation itself, as it's typically run locally or within a controlled environment. Omniverse authentication might be handled by `isaaclab.sh`.

---

*Architecture analysis: 2026-03-26*
