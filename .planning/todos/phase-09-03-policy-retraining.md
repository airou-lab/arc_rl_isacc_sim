# Phase 09-03: Policy Retraining

## Goal
Train the PPO agent to handle the high-inertia 1500kg 8x scale physics in the stabilized environment.

## Current Setup
- **Environments:** `--num_envs 2` (RTX 3060 memory limit).
- **Policy:** `MlpPolicy` (Flattened telemetry + Visuals).
- **Observations:** 12-float telemetry (Indices 3,4,8,9,11 active).
- **Physics:** 1500kg mass, 50k stiffness actuators.
- **Lane Limit:** 0.6m drift (Terminates on boundary).

## Tasks
- [ ] **Task 1: Initial Training Run**
  - Execute `bash train.sh` with the new physics parameters.
  - Monitor convergence (Mean reward should increase as it learns to stay centered).
- [ ] **Task 2: Reward Shaping (If needed)**
  - Adjust `lateral_error` weight if agent struggles to stay in lane.
  - Tune `speed_reward` to encourage faster but safe navigation.
- [ ] **Task 3: Visual Validation**
  - Run `bash run_gui_verify.sh` with the newly trained `.pth` model.
  - Confirm the agent can complete at least one full lap.
