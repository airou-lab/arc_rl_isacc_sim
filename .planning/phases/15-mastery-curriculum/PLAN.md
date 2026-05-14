---
phase: 15-mastery-curriculum
plan: 01
type: execute
wave: 1
depends_on: []
files_modified: [arcproLab/policy_stack/policies/fusion_policy.py, arcproLab/mdp/track_manager.py, arcproLab/mdp/observations.py, arcproLab/mdp/rewards.py, arcproLab/mdp/terminations.py, arcproLab/scripts/train_policy.py, arcproLab/arcpro_env_cfg.py]
autonomous: true
requirements: [REQ-MASTERY-BOUND, REQ-MASTERY-LAP, REQ-MASTERY-RAM, REQ-MASTERY-MONITOR]

must_haves:
  truths:
    - "Vision backbone uses ResNet-18 with ImageNet weights (unfrozen) at 224x224"
    - "TrackManager uses persistent windowed search (+/- 50 indices) for waypoint tracking"
    - "Lateral reward features a 10cm plateau before dropping off"
    - "Training pipeline scales to 32 environments with uint8 observation storage"
  artifacts:
    - path: "arcproLab/policy_stack/policies/fusion_policy.py"
      provides: "ResNet-18 backbone with ImageNet-v1 weights"
    - path: "arcproLab/mdp/track_manager.py"
      provides: "Index-persistent windowed search with +/- 50 index bounds"
    - path: "arcproLab/mdp/rewards.py"
      provides: "Plateau reward (+/- 10cm) and Jerk penalty (-100.0)"
    - path: "arcproLab/arcpro_env_cfg.py"
      provides: "32-environment config with 224x224 camera resolution"
  key_links:
    - from: "arcproLab/mdp/observations.py"
      to: "arcproLab/mdp/track_manager.py"
      via: "Gate distance (dist_g) retrieval"
    - from: "arcproLab/scripts/train_policy.py"
      to: "arcproLab/policy_stack/policies/fusion_policy.py"
      via: "uint8-to-float normalization in forward pass"
---

<objective>
Unify the ARCPro RL training pipeline for Mastery-level lane following. This plan implements the finalized parameters for Phase 15: Single-Agent Mastery. It upgrades the perception to ResNet-18, stabilizes waypoint tracking with windowed search, and tunes rewards/terminations for production-grade stability while scaling to 32 parallel environments.

Purpose: Full-Lap (6000 step) mastery with stable steering and line avoidance.
Output: ResNet-18 vision pipeline, windowed tracking, mastery reward structure, and 32-env production config.
</objective>

<execution_context>
@$HOME/.gemini/get-shit-done/workflows/execute-plan.md
@$HOME/.gemini/get-shit-done/templates/summary.md
</execution_context>

<context>
@.planning/ROADMAP.md
@.planning/STATE.md
@arcproLab/policy_stack/policies/fusion_policy.py
@arcproLab/mdp/track_manager.py
@arcproLab/mdp/observations.py
@arcproLab/mdp/rewards.py
@arcproLab/mdp/terminations.py
@arcproLab/scripts/train_policy.py
@arcproLab/arcpro_env_cfg.py
</context>

<tasks>

<task type="auto">
  <name>Task 1: Perception & Robust Logic (Wave 1)</name>
  <files>arcproLab/policy_stack/policies/fusion_policy.py, arcproLab/mdp/track_manager.py, arcproLab/mdp/observations.py</files>
  <action>
    - **Vision Backbone**: In `fusion_policy.py`, replace the custom CNN in `FusionFeaturesExtractor` with `torchvision.models.resnet18(weights='IMAGENET1K_V1')`.
      - Remove the final `fc` layer of ResNet and feed the 512-dim output into the `fusion_head`.
      - ResNet weights must be unfrozen (default in PyTorch).
      - Ensure the forward pass handles `uint8` inputs by casting to float and dividing by 255.0.
    - **Windowed Search**: Refactor `TrackManager.compute_errors` in `track_manager.py` to use Windowed Search.
      - Maintain a `self.last_indices` tensor (initialized to 0) to track the closest waypoint per environment.
      - Search for the closest waypoint within a +/- 50 index window of `last_indices`.
      - Reset window to full search if `episode_length_buf == 0` (handled via an external reset signal or checking env state).
    - **Gate-Awareness**: In `observations.py`, retrieve `dist_g` from `tm.compute_marker_distances`.
      - Store `dist_g` in `env.extras["dist_g"]` so it can be used to mask line penalties in rewards/terminations.
  </action>
  <verify>
    <automated>python3 -c "import torch; from torchvision.models import resnet18; m = resnet18(weights='IMAGENET1K_V1'); print('ResNet-18 Loaded')"</automated>
  </verify>
  <done>ResNet-18 is implemented, TrackManager uses windowed search, and gate distance is exposed for masking.</done>
</task>

<task type="auto">
  <name>Task 2: Mastery Rewards & Environment Scaling (Wave 2)</name>
  <files>arcproLab/mdp/rewards.py, arcproLab/mdp/terminations.py, arcproLab/scripts/train_policy.py, arcproLab/arcpro_env_cfg.py</files>
  <action>
    - **Mastery Rewards**: Update `rewards.py`.
      - **Plateau Reward**: Modify `lateral_error_reward` to provide a constant 1.0 within +/- 10cm of centerline, then drop off linearly or exponentially.
      - **Jerk Penalty**: Implement a penalty of -100.0 for high action rates (specifically steering jitter).
    - **Permissive Terminations**: Update `white_line_contact` in `terminations.py`.
      - Change the reset margin to 0.05m (allowing closer proximity to lines before resetting).
      - Use `env.extras["dist_g"]` to mask the boundary reset if within 0.20m of a gate.
    - **Production Scaling**: 
      - **uint8 Storage**: In `train_policy.py`, ensure the observation space for images is `np.uint8` and the bridge converts incoming tensors to `uint8` to save VRAM/RAM.
      - **32 Envs**: In `arcpro_env_cfg.py`, set `num_envs=32` and `width=224, height=224` for the `tiled_camera`.
  </action>
  <verify>
    <automated>grep "num_envs=32" arcproLab/arcpro_env_cfg.py</automated>
  </verify>
  <done>Reward structure is tuned for mastery, terminations are permissive for gate crossing, and pipeline scales to 32 envs.</done>
</task>

</tasks>

<verification>
Run `python3 arcproLab/scripts/train_policy.py --num_envs 32 --headless` for 1000 steps and verify:
1. No OOM errors with 32 environments at 224x224.
2. `FusionFeaturesExtractor` correctly processes ImageNet-weighted ResNet-18 features.
3. TrackManager persistent indices update correctly without "snapping" to far-away track segments.
</verification>

<success_criteria>
- ResNet-18 model initializes with ImageNet weights and processes 224x224 images.
- Agent survives 6000+ step laps with stable steering.
- uint8 storage and 32-env scaling enables high-throughput training on consumer GPUs.
- Windowed search prevents waypoint failures on hairpins.
</success_criteria>

<output>
After completion, create `.planning/phases/15-mastery-curriculum/15-01-SUMMARY.md`
</output>
