# Integrations

## Internal Integrations
- **USD Scene Configuration:** `arcproLab/arcpro_env_cfg.py` integrates `openStreetUSD` assets.
- **Track Management:** `mdp/track_manager.py` samples waypoints from USD meshes at runtime.
- **Observation Pipeline:** `mdp/observations.py` pulls data from the `Articulation` and `TiledCamera` buffers.

## External Integrations
- **ROS 2 Bridge:** Maps `/camera/image_raw`, `/vehicle_state`, and `/ackermann_cmd` to simulation objects.
- **Policy Storage:** Supports Stable Baselines 3 `.zip` format loading.
- **CI/CD:** GitHub Actions workflow (`ci.yml`) for linting and syntax validation.
