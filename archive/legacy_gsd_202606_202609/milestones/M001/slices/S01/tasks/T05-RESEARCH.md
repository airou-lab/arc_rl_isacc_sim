# Phase 16 SKRL & Asymmetric Actor-Critic (AAC) Research

## Current State
- Training using End-to-End PPO with `stable-baselines3` (SB3) is struggling to learn, constantly hitting maximum KL divergence limits (`max kl: 0.05`), leading to early stopping and crawling local minimums.
- The previous agent documented Decision **D007** (unfreezing ResNet) but also logged a high-confidence finding recommending an Asymmetric Actor-Critic (AAC) architecture.
- In AAC, the Actor uses only visual observations (camera images), while the Critic is fed privileged ground-truth state information (e.g., global position, velocity, track bounds) to massively improve sample efficiency and value estimation during training.
- SB3 does not natively support Asymmetric Actor-Critic without significant custom routing and engineering complexity.
- Our immediate next steps (from `.continue-here`) involve multi-agent RL (MARL).

## Technical Requirements for SKRL Integration
- **Framework Replacement**: Transition from `stable-baselines3` to `skrl`.
- **SKRL Advantages**: `skrl` is built natively for PyTorch, integrates seamlessly with Isaac Lab, and natively supports Multi-Agent RL (MARL) algorithms (e.g., MAPPO, IPPO) and Asymmetric Actor-Critic.
- **Environment Wrappers**: We will need to wrap our Isaac Lab `ManagerBasedRLEnv` with SKRL's Isaac Lab wrapper.
- **Model Definition**: Define separate PyTorch `nn.Module` classes for the Actor (using the ResNet-18 vision backbone) and the Critic (using an MLP to process the privileged `state`).

## Codebase Impact
1. **Training Script**: `arcproLab/scripts/train_policy.py` currently relies on SB3's `PPO` and custom callbacks. It will need a complete rewrite to instantiate `skrl` agents, models, and trainers.
2. **Environment config**: `arcpro_env_cfg.py` must ensure it exposes a privileged `state` tensor (in addition to `obs`) which `skrl` can route to the Critic.
3. **Dependencies**: Ensure `skrl` is added to `requirements.txt`.
