---
phase: 01-lane-environment-setup
plan: 01
type: execute
wave: 1
depends_on: []
files_modified:
  - openStreetUSD/arcpro_RL_open_street_sim.usd
  - arcproLab/mdp/track_centerline.npy
  - arcproLab/arcpro_env_cfg.py
autonomous: true
requirements:
  - REQ-ENV-LANE

must_haves:
  truths:
    - "The Isaac Sim environment visibly displays a two-lane track when loaded."
    - "The track geometry defines a clear centerline that the robot can follow."
    - "The track data (track_centerline.npy) accurately represents the track's centerline and boundaries."
  artifacts:
    - path: "openStreetUSD/arcpro_RL_open_street_sim.usd"
      provides: "Visual and physical representation of the two-lane track"
      contains: "UsdGeom.Mesh" # Indicating actual geometry, not just an empty file
    - path: "arcproLab/mdp/track_centerline.npy"
      provides: "NumPy array with track centerline waypoints"
      exists: true
      type: "NumPy array"
      shape: "(N, 3)"
  key_links:
    - from: "openStreetUSD/arcpro_RL_open_street_sim.usd"
      to: "Isaac Sim renderer"
      via: "USD stage loading"
      pattern: "USD file loads correctly in Isaac Sim"
    - from: "arcproLab/generate_track.py"
      to: "openStreetUSD/arcpro_RL_open_street_sim.usd"
      via: "USD parsing and sampling"
      pattern: "script successfully extracts waypoints from USD"
    - from: "arcproLab/arcpro_env_cfg.py"
      to: "openStreetUSD/arcpro_RL_open_street_sim.usd"
      via: "UsdFileCfg path"
      pattern: "UsdFileCfg\(usd_path="openStreetUSD/arcpro_RL_open_street_sim.usd"\)"
---

<objective>
Establish a basic two-lane track in the Isaac Sim environment and generate its corresponding waypoint data.

Purpose: This plan lays the foundational environment for the robot to operate within, providing the visual and data-driven representation of the track necessary for navigation and training.
Output: A modified USD scene file with a visible two-lane track and a generated `.npy` file containing the track's centerline waypoints.
</objective>

<execution_context>
@~/.gemini/get-shit-done/workflows/execute-plan.md
@~/.gemini/get-shit-done/templates/summary.md
</execution_context>

<context>
@.planning/PROJECT.md
@.planning/ROADMAP.md
@.planning/STATE.md
@.planning/codebase/ARCHITECTURE.md
@.planning/codebase/STACK.md
@.planning/codebase/STRUCTURE.md
@.planning/codebase/CONVENTIONS.md

# Existing USD scene file (for modification)
@openStreetUSD/arcpro_RL_open_street_sim.usd
</context>

<tasks>

<task type="auto">
  <name>Task 1: Define a simple two-lane track in USD</name>
  <files>openStreetUSD/arcpro_RL_open_street_sim.usd</files>
  <action>
    Modify `openStreetUSD/arcpro_RL_open_street_sim.usd` to include a simple two-lane track.
    - Add a `UsdGeom.Plane` or similar primitive to represent the road.
    - Add `UsdGeom.Mesh` or `UsdGeom.Cylinder` primitives to define two parallel lines as lane markers on the plane.
    - Ensure the scene is visually clear and identifiable as a two-lane track.
    - Avoid complex geometries or textures for this initial setup to keep it simple and easy to parse for `generate_track.py`.
    - Set the units to meters (per project conventions if not already).
  </action>
  <verify>
    <automated># Manual verification is needed to inspect the USD scene visually.
# The `run_gui_verify.sh` script can be modified to load this specific USD.
# For now, verification is file content based.
grep -q "UsdGeom.Mesh" openStreetUSD/arcpro_RL_open_street_sim.usd && grep -q "UsdGeom.Plane" openStreetUSD/arcpro_RL_open_street_sim.usd</automated>
  </verify>
  <done>
    `openStreetUSD/arcpro_RL_open_street_sim.usd` contains definitions for a road surface and two distinct lane markers.
  </done>
</task>

<task type="auto">
  <name>Task 2: Generate track centerline waypoints from USD</name>
  <files>arcproLab/mdp/track_centerline.npy</files>
  <action>
    Execute the `arcproLab/generate_track.py` script.
    This script will:
    - Load the `openStreetUSD/arcpro_RL_open_street_sim.usd` scene.
    - Sample waypoints representing the track's centerline.
    - Save these waypoints as a NumPy array to `arcproLab/mdp/track_centerline.npy`.
    Ensure the script runs successfully and creates the `.npy` file.
    No modifications to `generate_track.py` are expected in this task unless it fails to run against the newly created USD.
  </action>
  <verify>
    <automated>python3 -c "import numpy as np; data = np.load('arcproLab/mdp/track_centerline.npy'); assert data.shape[1] == 3 and data.shape[0] > 0, 'Invalid track_centerline.npy format or empty'; print('track_centerline.npy is valid.')"</automated>
  </verify>
  <done>
    A valid `arcproLab/mdp/track_centerline.npy` file exists, containing a NumPy array of (N, 3) float values, where N is the number of waypoints and 3 represents (x, y, z) coordinates.
  </done>
</task>

<task type="auto">
  <name>Task 3: Verify environment configuration references the USD</name>
  <files>arcproLab/arcpro_env_cfg.py</files>
  <action>
    Ensure that the `ARCProSceneCfg` within `arcproLab/arcpro_env_cfg.py` correctly points to the `openStreetUSD/arcpro_RL_open_street_sim.usd` file.
    Specifically, locate the `track` attribute (or similar, based on `ARCHITECTURE.md`) and verify its `usd_path` is set to the correct USD file.
    If the path is incorrect, update it to `"openStreetUSD/arcpro_RL_open_street_sim.usd"`.
    Do not modify other parts of the configuration.
  </action>
  <verify>
    <automated>grep -q 'usd_path="openStreetUSD/arcpro_RL_open_street_sim.usd"' arcproLab/arcpro_env_cfg.py</automated>
  </verify>
  <done>
    The `arcproLab/arcpro_env_cfg.py` file correctly configures the environment to load `openStreetUSD/arcpro_RL_open_street_sim.usd`.
  </done>
</task>

</tasks>

<verification>
After successfully completing all tasks:
1. Load the Isaac Sim environment using the configured `arcproLab/arcpro_env_cfg.py`.
2. Visually inspect the loaded scene to confirm the presence and correct appearance of the two-lane track.
3. Observe if the `TrackManager` successfully loads and utilizes the `track_centerline.npy` data without errors.
</verification>

<success_criteria>
- The Isaac Sim environment launches without errors and displays a clearly defined two-lane track.
- The `arcproLab/mdp/track_centerline.npy` file is generated and contains valid waypoint data for the track.
- The `arcproLab/arcpro_env_cfg.py` correctly references the track's USD file.
</success_criteria>

<output>
After completion, create `.planning/phases/01-lane-environment-setup/01-lane-environment-setup-01-SUMMARY.md`
</output>
