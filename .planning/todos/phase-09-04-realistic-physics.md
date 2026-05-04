# TODO: Realistic Physics Refinement (5kg Mass)

Reduce the robot mass from the "stability anchor" of 30kg to a realistic F1Tenth mass of 5kg while maintaining physical stability.

## Tasks
- [ ] **Mass Reduction:** Update `arcpro_robot_cfg.py` to set Chassis mass to 5.0kg.
- [ ] **Torque Scaling:** Reduce motor `effort_limit` and `ActionCfg` scale to match the 83% reduction in mass.
- [ ] **Damping Calibration:** Fine-tune joint damping to prevent high-RPM wheel spin-up with the lighter load.
- [ ] **Verification:** Run `verify_drive.py` in GUI mode to ensure the car does not flip or "pop wheelies" under full throttle.

## Rationale
The 30kg mass was a necessary hack to stabilize the simulation during the initial 1.0x metric scale transition. To ensure the model is transferable to real-world hardware, the physics must reflect realistic 1/10th scale proportions.
