---
phase: 15-mastery-curriculum
plan: 01
type: execute
wave: 1
depends_on: []
files_modified: [arcproLab/policy_stack/policies/fusion_policy.py, arcproLab/arcpro_env_cfg.py, arcproLab/mdp/track_manager.py, arcproLab/mdp/rewards.py, arcproLab/scripts/train_policy.py, arcproLab/policy_stack/wrappers/curriculum_callback.py]
autonomous: true
requirements: [REQ-MASTERY-BOUND, REQ-MASTERY-LAP, REQ-MASTERY-RAM, REQ-MASTERY-MONITOR]

must_haves:
  truths:
    - "Vision backbone uses ResNet-18 at 224x224 resolution"
    - "TrackManager uses windowed search to prevent hairpin snaps"
    - "Lateral error reward uses an exponential 'Magnetic' centerline curve"
    - "RolloutBuffer uses uint8 storage to support 32 parallel environments"
  artifacts:
    - path: "arcproLab/policy_stack/policies/fusion_policy.py"
      provides: "ResNet-18 backbone and uint8-to-float normalization"
    - path: "arcproLab/mdp/track_manager.py"
      provides: "Windowed waypoint search with index persistence"
    - path: "arcproLab/mdp/rewards.py"
      provides: "Magnetic centerline reward and Gate-Aware soft-boundary penalties"
    - path: "arcproLab/arcpro_env_cfg.py"
      provides: "224x224 resolution and 32-env production config"
  key_links:
    - from: "arcproLab/scripts/train_policy.py"
      to: "arcproLab/policy_stack/policies/fusion_policy.py"
      via: "uint8 observation space"
    - from: "arcproLab/mdp/rewards.py"
      to: "arcproLab/mdp/track_manager.py"
      via: "Gate distance masking"
---

<objective>
Unify the ARCPro RL training pipeline for Mastery-level lane following. This plan pivots the vision backbone to ResNet-18 @ 224x224, implements robust windowed tracking to prevent search failures, and optimizes memory to scale to 32 parallel environments using uint8 storage.

Purpose: Achieve 6000+ step survival (full lap) with production-grade stability and vision.
Output: ResNet-18 vision pipeline, windowed track management, and magnetic reward structure.
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
@arcproLab/mdp/rewards.py
@arcproLab/arcpro_env_cfg.py
</context>

<tasks>

<task type="auto">
  <name>Task 1: Perception Backbone & Windowed Logic (Wave 1)</name>
  <files>arcproLab/policy_stack/policies/fusion_policy.py, arcproLab/arcpro_env_cfg.py, arcproLab/mdp/track_manager.py</files>
  <action>
    - **Vision Pivot**: In `fusion_policy.py`, replace the custom CNN in `FusionFeaturesExtractor` with `torchvision.models.resnet18(weights=None)`.
      - Remove the final `fc` layer of ResNet and feed the 512-dim output into the `fusion_head`.
      - Ensure the extractor handles `uint8` inputs by casting to float and dividing by 255.0 at the start of `forward`.
    - **Resolution Sync**: Update `arcpro_env_cfg.py` to set `width=224, height=224` for the `tiled_camera`.
    - **Windowed Search**: Update `TrackManager.compute_errors` in `track_manager.py` to implement windowed searching.
      - Maintain a `self.last_indices` tensor of size `[max_envs]`.
      - For each environment, search for the closest waypoint within a +/- 100 index window of `last_indices[env]`.
      - Reset window to full search if `env.episode_length_buf == 0`.
  </action>
  <verify>
    <automated>python3 -c "import torch; from torchvision.models import resnet18; m = resnet18(); print(m.fc)"</automated>
  </verify>
  <done>Backbone is ResNet-18, resolution is 224x224, and TrackManager uses persistent windows for waypoint search.</done>
</task>

<task type="auto">
  <name>Task 2: Mastery Rewards & Masking (Wave 1)</name>
  <files>arcproLab/mdp/rewards.py, arcproLab/mdp/terminations.py</files>
  <action>
    - **Magnetic Reward**: In `rewards.py`, update `lateral_error_reward` to use an exponential curve: `exp(-20.0 * (lat_err^2))`. This provides a sharp peak at the centerline and a smooth gradient across the lane.
    - **Soft-Boundary Penalty**: Implement `soft_boundary_penalty` in `rewards.py`.
      - Calculate distance to boundaries using `tm.compute_marker_distances`.
      - Apply penalty if `dist < 0.25m` (Soft Boundary).
      - **Gate-Aware Masking**: Mask the penalty if `dist_gate < 0.20m` (Permeable Gate).
    - **Termination Tuning**: Update `white_line_contact` in `terminations.py` to use a strict `0.13m` threshold (Hard Boundary).
  </action>
  <verify>
    <automated>grep "exp" arcproLab/mdp/rewards.py</automated>
  </verify>
  <done>Reward structure encourages centerline mastery and penalizes boundary proximity unless crossing a gate.</done>
</task>

<task type="auto">
  <name>Task 3: Production Scaling & Monitoring (Wave 2)</name>
  <files>arcproLab/scripts/train_policy.py, arcproLab/arcpro_env_cfg.py, arcproLab/policy_stack/wrappers/curriculum_callback.py</files>
  <action>
    - **uint8 Optimization**: In `train_policy.py`, update `HPPPDirectBridge` to define the `image` observation space as `dtype=np.uint8`.
      - Ensure `_process_obs` converts float images to `uint8` (0-255).
    - **Environment Scaling**: In `arcpro_env_cfg.py`, set `num_envs=32`.
    - **Hyperparam Update**: Update `train_policy.py` for 32 envs:
      - `n_steps=1024` (High throughput).
      - `batch_size=128`.
      - `learning_rate=1e-4` (Stability).
    - **Curriculum Callback**: Create `curriculum_callback.py` to monitor and log `curriculum/max_episode_length` and `curriculum/mean_reward` to Tensorboard.
  </action>
  <verify>
    <automated>grep "num_envs=32" arcproLab/arcpro_env_cfg.py</automated>
  </verify>
  <done>Training pipeline is optimized for 32 envs with uint8 storage and automated monitoring.</done>
</task>

</tasks>

<verification>
Run `python3 arcproLab/scripts/train_policy.py --num_envs 32 --headless` for 5000 steps and verify:
1. No OOM errors.
2. Tensorboard logs appear for curriculum metrics.
3. VRAM usage < 12GB.
</verification>

<success_criteria>
- ResNet-18 model converges on lane following within 1M steps.
- Agent survives full laps (6000+ steps) at 1.5 m/s.
- uint8 storage reduces RolloutBuffer RAM usage by 4x.
- Windowed search prevents waypoint "snapping" at turns.
</success_criteria>

<output>
After completion, create `.planning/phases/15-mastery-curriculum/15-01-SUMMARY.md`
</output>
