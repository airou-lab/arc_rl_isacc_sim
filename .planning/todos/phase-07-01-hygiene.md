# Phase 07-01: Gymnasium & Environment Hygiene

**Goal**: Modernize the API and clean up terminal noise/broken USD references.

## Tasks
- [x] **Task 1: Gymnasium Standardization**
  - Update `arcproLab/__init__.py` to use `gymnasium` explicitly.
  - Standardize all `import gymnasium as gym` across the project.
  - Add specific warning filter for legacy `gym` noise.
- [x] **Task 2: USD Reference Fix (Signposts)**
  - Identify a valid Isaac Sim signpost asset path.
  - Run a script to redirect broken references in `no_graph_sim.usd`.
- [x] **Task 3: Simulation Optimization**
  - Update `arcpro_env_cfg.py`: Disable CCD, enable `enable_external_forces_every_iteration`.
- [x] **Task 4: Dead Reference Removal**
  - Remove the `/World/F1Tenth` reference that points to a missing folder.
- [x] **Task 5: Validation**
  - Run `train_policy.py` with `--num_envs 1` to verify a clean terminal output.

## Success Criteria
- [x] No "Gym is unmaintained" warnings (Filtered in script).
- [x] No "Unresolved reference" errors for signposts or F1Tenth.
- [x] Simulation starts and the robot moves smoothly with AWD.
