# Todo: Visual Intersection Mastery - Stop Signs

## Description
Enhance the training environment with visual cues for intersections. This task involves sourcing stop sign assets from Isaac Sim prims, scaling them down to the robot's metric scale (1.0x/F1Tenth scale), and placing them at known intersections on the track.

## Success Criteria
- [ ] Source a high-quality stop sign prim from Isaac Sim.
- [ ] Create a `StopSignCfg` in `arcproLab/mdp/spawner.py` or `arcpro_env_cfg.py`.
- [ ] Shrunk sign to robot scale (approx 0.15m height for the pole/sign to match F1Tenth perspective).
- [ ] Place signs at intersection entries in the `no_graph_sim_clean_1x.usda` world.
- [ ] Verify that the 640x360 HD camera can visually distinguish the red octagon.

## Related
- Milestone: Milestone 3
- Phase: Phase 15
- Architecture: HD Vision + Adaptive CNN
