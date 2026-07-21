# T05: SKRL AAC Integration Plan

**Goal:** Refactor the training pipeline to replace `stable-baselines3` with `skrl` and implement an Asymmetric Actor-Critic (AAC) architecture.

## Phase 1: Environment Preparation (Privileged State)
- **Task:** Update `arcpro_env_cfg.py` and `arcproLab/mdp/observations.py` to define a privileged `state` observation group.
- **Details:** The `state` should include exact vehicle poses, velocities, distances to lane centerlines, and upcoming turn tokens. This will be the input to the Critic. The standard `obs` will remain the camera image for the Actor.

## Phase 2: SKRL Model Definition
- **Task:** Create `arcproLab/agents/skrl_models.py`.
- **Details:**
  - Implement a `Shared` or separate `Actor` and `Critic` classes inheriting from `skrl.models.torch.Model`.
  - **Actor:** Incorporates the ResNet-18 vision backbone.
  - **Critic:** An MLP that ingests the privileged `state`.

## Phase 3: Training Script Migration
- **Task:** Rewrite `arcproLab/scripts/train_policy.py` (or create `train_skrl.py`).
- **Details:** 
  - Remove all `stable-baselines3` imports.
  - Wrap the Isaac Lab environment using `skrl.envs.wrappers.torch.IsaacLabWrapper`.
  - Initialize the `PPO` (or `MAPPO`) agent from `skrl.agents.torch.ppo`.
  - Configure the agent to use Asymmetric Actor-Critic (routing `state` to the critic).
  - Setup logging, checkpointing, and tensorboard using `skrl` utilities.

## Phase 4: Testing & Verification
- **Task:** Run a short headless test to verify tensors are routed correctly and training loop does not crash. Ensure KL divergence stabilizes compared to the SB3 implementation.
