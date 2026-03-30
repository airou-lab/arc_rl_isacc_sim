# Codebase Concerns

**Analysis Date:** 2024-11-20

## Tech Debt

**Hardcoded Environment Paths:**
- Issue: Several shell scripts hardcode absolute paths to the Isaac Lab executable and the project directory, specifically referencing `/home/arika/...`.
- Files:
  - `train.sh`
  - `verify_sim.sh`
  - `verify_sim_metric.sh`
  - `run_gui_verify.sh`
- Impact: Makes the codebase non-portable and causes failures for any other developer or CI/CD runner.
- Fix approach: Use environment variables or relative path discovery (e.g., `$(pwd)` or finding `isaaclab.sh` in the parent directories).

**Unpinned Dependencies:**
- Issue: `requirements.txt` lacks version pinning for most packages (except `numpy < 2`).
- Files: `requirements.txt`
- Impact: Risk of breaking changes from future package updates, lack of reproducibility across environments, and potential security risks.
- Fix approach: Pin exact versions using `pip freeze > requirements.txt` after a known-good installation.

**Unorganized `trash/` Directory:**
- Issue: The `trash/tools/` directory contains 94 scripts with no documentation or clear organization.
- Files: `trash/tools/*`
- Impact: Bloats the repository and makes it difficult to find truly useful utility scripts.
- Fix approach: Audit the directory, move useful scripts to a properly organized `arcproLab/utils/` directory, and delete the rest.

**Magic Numbers and Hardcoded Offsets:**
- Issue: Widespread use of hardcoded values for reward weights, termination limits, and steering geometry calculations.
- Files:
  - `arcproLab/mdp/rewards.py` (e.g., `0.3`, `0.5`, `2.0`, `-10.0`)
  - `arcproLab/mdp/terminations.py` (e.g., `0.02`, `0.3`)
  - `arcproLab/mdp/policy_wrapper.py` (e.g., `15.0`, `40.0`, `1.0`, `0.05`)
  - `arcproLab/mdp/track_manager.py` (fallback coordinates `-125.0`, `62.0`)
- Impact: Makes the logic fragile and difficult to tune or adapt to new robots/maps.
- Fix approach: Move these values to configuration files (`arcpro_env_cfg.py` or separate YAML/JSON configs).

## Security Considerations

**Hardcoded Absolute Paths:**
- Risk: Exposes the local machine's directory structure and user names in the repository.
- Files: `train.sh`, `verify_sim.sh`, `verify_sim_metric.sh`, `run_gui_verify.sh`
- Current mitigation: None.
- Recommendations: Replace with environment variables or relative paths.

**Unpinned Dependencies:**
- Risk: Potential for "dependency confusion" attacks or inclusion of malicious versions of libraries.
- Files: `requirements.txt`
- Current mitigation: None.
- Recommendations: Pin dependencies and use a lockfile (e.g., `poetry.lock` or `Pipfile.lock`).

## Performance Bottlenecks

**Track Manager Sampling Logic:**
- Problem: `TrackManager.sample_waypoints_from_usd` traverses the entire USD stage, which can be slow for very large scenes.
- Files: `arcproLab/mdp/track_manager.py`
- Cause: Iterative searching of all meshes in the stage.
- Improvement path: Optimize the search by specifying the parent prim for road meshes or cache the results (currently saves to `.npy`, which is good, but sampling is still a potential bottleneck on first run).

## Fragile Areas

**USD Sampling Heuristics:**
- Files: `arcproLab/mdp/track_manager.py`
- Why fragile: Relies on string-based keyword matching (e.g., `pavement`, `road`, `track`, `drivable_surfaces`) for identifying road meshes. If a map uses different naming conventions, it will fail to generate waypoints correctly.
- Safe modification: Use USD attributes or tags instead of name-based filtering.
- Test coverage: Gaps in verifying sampling correctness across different maps.

**Policy Wrapper Normalization:**
- Files: `arcproLab/mdp/policy_wrapper.py`
- Why fragile: Uses ImageNet normalization constants (`[0.485, 0.456, 0.406]`, `[0.229, 0.224, 0.225]`), which may not match the distribution of the simulation's visual data.
- Safe modification: Compute normalization statistics from the actual training dataset.

## Missing Critical Features

**Lack of Centralized Utilities:**
- Problem: No `utils/` or `tools/` directory for shared helper functions, leading to logic duplication or scripts being hidden in `trash/`.
- Blocks: Better code organization and reuse.

## Test Coverage Gaps

**Untested Core Logic:**
- What's not tested: Reward functions, termination criteria, observation managers, and policy inference logic.
- Files:
  - `arcproLab/mdp/rewards.py`
  - `arcproLab/mdp/terminations.py`
  - `arcproLab/mdp/observations.py`
  - `arcproLab/mdp/policy_wrapper.py`
- Risk: Regressions or logic errors could go unnoticed during development, especially when tuning physics or reward parameters.
- Priority: High

---

*Concerns audit: 2024-11-20*
