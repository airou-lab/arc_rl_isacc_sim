# Architecture

**Analysis Date:** 2024-05-24

## Pattern Overview

**Overall:** Hierarchical Deep Reinforcement Learning with Isaac Lab (Manager-Based RL)

**Key Characteristics:**
- **Simulation Engine:** Powered by NVIDIA Omniverse Isaac Sim and managed via the Isaac Lab framework.
- **Hierarchical Agent:** Uses a Driver-Worker architecture where a classical planner (`WorkerNode`) decides "where to go" topologically, and a learned neural policy (`MainNode`) handles visual-motor "how to get there."
- **Multi-Modal Perception:** Combines high-dimensional visual input (processed via an early-pooled ResNet-18 backbone) with low-dimensional physical telemetry.

## Layers

**Simulation Layer:**
- Purpose: Defines the physical world, rendering settings, and asset configurations using PhysX 5.
- Location: `arcproLab/`
- Contains: Environment definition (`arcpro_env_cfg.py`) and robot configuration (`arcpro_robot_cfg.py`).
- Depends on: Isaac Lab (`isaaclab.envs.ManagerBasedRLEnvCfg`), PyTorch.
- Used by: Training and evaluation scripts.

**MDP (Markov Decision Process) Layer:**
- Purpose: Defines the RL problem structure modularly.
- Location: `arcproLab/mdp/`
- Contains: State functions (`observations.py`), environment reactions (`events.py`), penalties/incentives (`rewards.py`), terminal conditions (`terminations.py`), and action space mappings (`actions.py`).
- Depends on: Isaac Lab `isaaclab.managers`, PyTorch.
- Used by: The Simulation Layer configuration.

**Policy Layer:**
- Purpose: Holds the neural network backbone, training wrappers, and high-level behavioral logic.
- Location: `arcproLab/policy_stack/`
- Contains: The neural network extractor (`policies/fusion_policy.py`), SB3 environment wrappers, and the hierarchical agent (`agent/agent_node.py`).
- Depends on: Stable-Baselines3, PyTorch, Torchvision.
- Used by: Training scripts.

## Data Flow

**Hierarchical Control Flow:**

1. **Observation Extraction:** The simulation layer extracts a raw observation consisting of a Camera Image and a Telemetry Vector.
2. **Topological Planning (WorkerNode):** The `WorkerNode` queries an intersection graph based on current world-frame coordinates. It computes a discrete turn command (`turn_token`) and requests permission from a multi-agent scheduler (`go_signal`).
3. **Observation Enrichment:** The `turn_token` and `go_signal` are injected into the first two indices of the observation telemetry vector.
4. **Policy Forward Pass:** SB3's `RecurrentPPO` processes the enriched observation via `FusionFeaturesExtractor`. The image passes through a ResNet-18 backbone and concatenates with the telemetry vector to output raw control signals.
5. **Safety Override (MainNode):** The `MainNode` applies a safety gate to the raw action. If the `go_signal` indicates wait, it forcibly zeros the throttle and applies the brake. Otherwise, the learned action is preserved.
6. **Action Application:** The final action is mapped to vehicle joint commands by the MDP action manager.

**State Management:**
- Physics state is managed by Isaac Sim (PhysX).
- Policy state (e.g., LSTM hidden states) is managed by Stable-Baselines3.
- Agent route state (e.g., intersection cooldown, current plan) is managed intrinsically within the `WorkerNode`.

## Key Abstractions

**Manager-Based Configuration:**
- Purpose: Declaratively maps logical environment components to the underlying Isaac Sim engine.
- Examples: `arcproLab/arcpro_env_cfg.py`
- Pattern: Configuration classes using the `@configclass` decorator.

**Agent Hierarchy:**
- Purpose: Separates discrete route planning from continuous motor control.
- Examples: `arcproLab/policy_stack/agent/agent_node.py`
- Pattern: The wrapper intercepts observations before the policy and intercepts actions after the policy, ensuring non-learned logic safely guides learned behavior.

**Fusion Features Extractor:**
- Purpose: Fuses multi-modal inputs in a VRAM-efficient manner.
- Examples: `arcproLab/policy_stack/policies/fusion_policy.py`
- Pattern: Custom `BaseFeaturesExtractor` that unfreezes only the deepest layers of an ImageNet-pretrained ResNet-18 backbone to balance rapid convergence with memory constraints.

## Entry Points

**Training Execution:**
- Location: `arcproLab/scripts/train_policy.py`
- Triggers: User execution from the CLI.
- Responsibilities: Bootstraps the Omniverse app, instantiates the Isaac Lab environment, wraps it in the custom `HPPPDirectBridge` VecEnv, configures SB3's `RecurrentPPO`, and initiates the training loop.

## Error Handling

**Strategy:** Immediate Episode Reset via Terminal Conditions

**Patterns:**
- **Boundary Collisions:** If the agent contacts track demarcations (`mdp/terminations.py`), an immediate reset is issued with a large penalty.
- **Stagnation:** Agents that get stuck and fall below a stationary threshold trigger an episode reset to prevent training stagnation.
- **Out of Bounds:** Height and field-of-view terminations catch agents that fly off the map or drive into unrenderable spaces.

## Cross-Cutting Concerns

**Logging:** Integrated via Stable-Baselines3's logging callbacks and Isaac Lab's internal wandb/tensorboard hooks.
**VRAM Efficiency:** Early pooling in CNN backbones and strict image data types (`uint8` in the replay buffer) are heavily enforced to mitigate the massive footprint of multi-agent HD rendering.
