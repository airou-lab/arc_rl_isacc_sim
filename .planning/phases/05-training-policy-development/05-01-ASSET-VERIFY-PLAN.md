---
phase: 05-training-policy-development
plan: 05-01-ASSET-VERIFY
type: execute
wave: 1
depends_on: []
files_modified: [
  "arcproLab/arcpro_env_cfg.py",
  "arcproLab/arcpro_robot_cfg.py",
  "openStreetUSD/",
  "arcproLab/assets/robot/"
]
autonomous: true
requirements: [REQ-SIM-METRIC]
must_haves:
  truths:
    - "Selected track asset has no broken sign references"
    - "Road surface collision mesh is continuous and stable"
    - "Track surface is grounded at Z=0.0 when offset by -1.25m"
    - "Robot car length is approximately 0.33m at 1.0x scale"
    - "Robot joints (steering and drive) are functional and articulate correctly"
  artifacts:
    - path: "openStreetUSD/no_graph_sim_final.usd"
      provides: "Definitive metric track"
    - path: "arcproLab/assets/robot/F1Tenth_Metric.usd"
      provides: "Definitive metric robot"
    - path: "arcproLab/arcpro_env_cfg.py"
      provides: "Environment configuration pointing to final assets"
    - path: "arcproLab/arcpro_robot_cfg.py"
      provides: "Robot configuration pointing to final assets"
  key_links:
    - from: "arcproLab/arcpro_env_cfg.py"
      to: "openStreetUSD/no_graph_sim_final.usd"
      via: "usd_path"
    - from: "arcproLab/arcpro_robot_cfg.py"
      to: "arcproLab/assets/robot/F1Tenth_Metric.usd"
      via: "usd_path"
---

<objective>
Verify and select the definitive USD assets for Phase 5. This plan ensures that the track and robot assets are physically accurate, properly grounded at a 1.0x metric scale, and free of unresolved references or integrity issues.

Purpose: Establish a stable and accurate asset foundation for policy training and hierarchical navigation.
Output: Verified "final" USD assets for track and robot, and updated environment configurations.
</objective>

<execution_context>
@$HOME/.gemini/get-shit-done/workflows/execute-plan.md
</execution_context>

<context>
@.planning/ROADMAP.md
@.planning/STATE.md
@arcproLab/arcpro_env_cfg.py
@arcproLab/arcpro_robot_cfg.py
@openStreetUSD/no_graph_sim_cleaned.usd
@openStreetUSD/no_graph_sim_final.usd
@arcproLab/assets/robot/F1Tenth_Metric.usd
</context>

<tasks>

<task type="auto">
  <name>Task 1: Track Asset Verification & Selection</name>
  <files>openStreetUSD/no_graph_sim_cleaned.usd, openStreetUSD/no_graph_sim_final.usd</files>
  <action>
    - Compare `no_graph_sim_cleaned.usd` vs `no_graph_sim_final.usd` using Isaac Sim tools or scripts (e.g., `trash/tools/inspect_collisions.py`).
    - Verify:
        - Unresolved asset references: Check for missing textures or sub-USDs (specifically broken signs).
        - Collision Mesh Integrity: Confirm the road surface is a valid collision mesh (not broken into disparate planes that cause physics jitter).
        - Grounding: Verify that applying a Z-offset of -1.25m (at 0.0825 scale) brings the road surface exactly to Z=0.0 in the world frame.
    - Select the asset that passes all checks as the definitive track.
  </action>
  <verify>
    <automated>python3 trash/tools/inspect_collisions.py --usd openStreetUSD/no_graph_sim_final.usd</automated>
  </verify>
  <done>Definitive track asset selected and verified for physical integrity and grounding.</done>
</task>

<task type="auto">
  <name>Task 2: Robot Asset Metricity & Articulation</name>
  <files>arcproLab/assets/robot/F1Tenth_Metric.usd</files>
  <action>
    - Verify `F1Tenth_Metric.usd` scale and dimensions:
        - Confirm the scale is 1.0x (metric).
        - Measure the total length of the car (should be approx 0.33m / 33cm).
    - Verify Articulation:
        - Check that wheels have valid collision meshes (cylinders or spheres).
        - Confirm that Joint_Steer_L/R and Joint_Drive_FL/FR/RL/RR are correctly defined as revolute joints.
    - Use `trash/tools/check_usd_meters.py` or similar to confirm metric scale.
  </action>
  <verify>
    <automated>python3 trash/tools/check_usd_meters.py --usd arcproLab/assets/robot/F1Tenth_Metric.usd</automated>
  </verify>
  <done>Robot asset verified as 1.0x metric scale with functional articulation.</done>
</task>

<task type="auto">
  <name>Task 3: Redundant Asset Cleanup & Configuration Finalization</name>
  <files>arcproLab/arcpro_env_cfg.py, arcproLab/arcpro_robot_cfg.py, openStreetUSD/, arcproLab/assets/robot/</files>
  <action>
    - Cleanup redundant track assets in `openStreetUSD/`:
        - Move `arcpro_RL_open_street_sim.usd`, `no_graph_sim.usd`, and the non-selected comparison asset to `trash/`.
    - Cleanup redundant robot assets in `arcproLab/assets/robot/`:
        - Move `F1Tenth_Generated.usd` to `trash/`.
    - Update `arcproLab/arcpro_env_cfg.py`:
        - Ensure `track.spawn.usd_path` points to the selected definitive asset (e.g., `no_graph_sim_final.usd`).
        - Confirm grounding offset `pos=(0,0,-1.25)` is active.
    - Update `arcproLab/arcpro_robot_cfg.py`:
        - Ensure `spawn.usd_path` points to `F1Tenth_Metric.usd`.
        - Confirm scale `(1.0, 1.0, 1.0)` is active.
  </action>
  <verify>
    <automated>./verify_sim_metric.sh --headless</automated>
  </verify>
  <done>Project directory cleaned of redundant assets and configurations finalized with definitive metric USDs.</done>
</task>

</tasks>

<success_criteria>
- Definitive USD assets selected and verified for metric accuracy.
- Redundant assets removed from active directories.
- Configurations point to verified final assets.
- Simulation launches without asset errors and maintains stable grounding.
</success_criteria>

<output>
After completion, create `.planning/phases/05-training-policy-development/05-01-ASSET-VERIFY-SUMMARY.md`
</output>
