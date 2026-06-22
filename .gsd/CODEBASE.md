# Codebase Map

Generated: 2026-06-22T15:01:04Z | Files: 130 | Described: 0/130
<!-- gsd:codebase-meta {"generatedAt":"2026-06-22T15:01:04Z","fingerprint":"8ccc39ea2b69732308f61e0225fcd350399a26db","fileCount":130,"truncated":false} -->

### (root)/
- `.continue-here`
- `.gitattributes`
- `.gitignore`
- `.gitmodules`
- `check_training_status.sh`
- `debug.sh`
- `fix_centerline.py`
- `fix_track.py`
- `inspect_usd.py`
- `relaunch_to_5M.sh`
- `relaunch_with_telemetry.sh`
- `requirements.txt`
- `run_train_bg.sh`
- `test_env_launcher.py`
- `test_env.py`

### .github/workflows/
- `.github/workflows/ci.yml`

### arcproLab/
- `arcproLab/__init__.py`
- `arcproLab/arcpro_env_cfg.py`
- `arcproLab/arcpro_robot_cfg.py`
- `arcproLab/generate_track.py`
- `arcproLab/policy_stack`
- `arcproLab/verify.py`

### arcproLab/assets/robot/
- `arcproLab/assets/robot/F1Tenth_Generated.usd`
- `arcproLab/assets/robot/F1Tenth_Metric.usd`

### arcproLab/mdp/
- `arcproLab/mdp/__init__.py`
- `arcproLab/mdp/actions.py`
- `arcproLab/mdp/debug_terminations.py`
- `arcproLab/mdp/events.py`
- `arcproLab/mdp/go_signal_manager.py`
- `arcproLab/mdp/observations_with_lat_err.py`
- `arcproLab/mdp/observations.py`
- `arcproLab/mdp/policy_wrapper.py`
- `arcproLab/mdp/rewards.py`
- `arcproLab/mdp/road_manager.py`
- `arcproLab/mdp/spawner.py`
- `arcproLab/mdp/terminations.py`
- `arcproLab/mdp/track_centerline.npy`
- `arcproLab/mdp/track_manager.py`
- `arcproLab/mdp/visual_analytics.py`

### arcproLab/models/
- `arcproLab/models/road_following_model.pth`

### arcproLab/scripts/
- *(41 files: 41 .py)*

### openStreetUSD/
- `openStreetUSD/no_graph_sim_1x.usda`
- `openStreetUSD/no_graph_sim_clean_1x_flattened.usda`
- `openStreetUSD/no_graph_sim_clean_1x.usda`
- `openStreetUSD/no_graph_sim_final.usd`
- `openStreetUSD/no_graph_sim.usd`
- `openStreetUSD/track_1x_wrapper.usda`

### openStreetUSD/archive/
- `openStreetUSD/archive/arcpro_RL_open_street_sim.usd`
- `openStreetUSD/archive/no_graph_sim_cleaned.usd`
- `openStreetUSD/archive/original_flattened.usd`
- `openStreetUSD/archive/original_hardened.usd`
- `openStreetUSD/archive/original_production_v2.usd`
- `openStreetUSD/archive/original_production.usd`
- `openStreetUSD/archive/original_thickened.usd`
- `openStreetUSD/archive/original_usd.usd`
- `openStreetUSD/archive/README.md`

### tensorboard_view_docker/
- `tensorboard_view_docker/docker-compose.yml`
- `tensorboard_view_docker/Dockerfile`

### trash/
- `trash/arcpro_marl_env_cfg.py`
- `trash/check_track.py`
- `trash/check_width.py`
- `trash/road_graph.py.bak`
- `trash/run_debug.sh`
- `trash/run_gui_verify.sh`
- `trash/run_train_test.sh`
- `trash/v2v_manager.py`

### trash/12-hierarchical-policy-migration/
- `trash/12-hierarchical-policy-migration/RESEARCH.md`

### trash/collapsed_models/
- `trash/collapsed_models/model_4000000_steps.zip`
- `trash/collapsed_models/model_5000000_steps.zip`

### trash/collapsed_models/20260507-192010/
- `trash/collapsed_models/20260507-192010/model_1000000_steps.zip`
- `trash/collapsed_models/20260507-192010/model_2000000_steps.zip`
- `trash/collapsed_models/20260507-192010/model_3000000_steps.zip`
- `trash/collapsed_models/20260507-192010/model_final_collapsed.zip.bak`
- `trash/collapsed_models/20260507-192010/vec_normalize.pkl`

### trash/collapsed_models/20260507-192010/tb/RecurrentPPO_1/
- `trash/collapsed_models/20260507-192010/tb/RecurrentPPO_1/events.out.tfevents.1778199611.airou-alienware.1115177.0`

### trash/legacy_docs/
- `trash/legacy_docs/PROJECT.md`
- `trash/legacy_docs/REQUIREMENTS.md`
- `trash/legacy_docs/RL_ISAAC_SIM.md`
- `trash/legacy_docs/RL_OVERVIEW.md`
- `trash/legacy_docs/RL_POLICY.md`
- `trash/legacy_docs/RL_REWARDS.md`
- `trash/legacy_docs/ROADMAP.md`
- `trash/legacy_docs/STATE.md`
- `trash/legacy_docs/SUMMARY.md`
- `trash/legacy_docs/VERIFICATION_HISTORY.md`

### trash/phase-15-old/
- `trash/phase-15-old/15-01-PLAN.md`
- `trash/phase-15-old/15-02-PLAN.md`
- `trash/phase-15-old/15-PLAN.md`

### trash/ppo/20260506-204037/tb/RecurrentPPO_1/
- `trash/ppo/20260506-204037/tb/RecurrentPPO_1/events.out.tfevents.1778118038.airou-alienware.2970877.0`

### trash/ppo/20260506-204422/tb/RecurrentPPO_1/
- `trash/ppo/20260506-204422/tb/RecurrentPPO_1/events.out.tfevents.1778118263.airou-alienware.3049072.0`
