# Decisions Register

<!-- Append-only. Never edit or remove existing rows.
     To reverse a decision, add a new row that supersedes it.
     Read this file at the start of any planning or research phase. -->

| # | When | Scope | Decision | Choice | Rationale | Revisable? | Made By |
|---|------|-------|----------|--------|-----------|------------|---------|
| D001 |  | reinforcement-learning | How to handle stagnation at curves caused by waypoint tangent target rewards. | Removed waypoint track rewards in favor of purely visual lateral error and heading rewards. Update speed_reward to just reward local forward velocity (-X). | The ResNet policy was being artificially penalized on curves because waypoint-based rewards wanted straight lines. Detaching reward from waypoints forces the vision backbone to learn the lane naturally without being hampered by geometry constraints. | Yes | agent |
| D002 |  | physics | How to align the coordinate frame for steering and lateral error. | Set Target Offset to 0.0m, Steering to Positive=Left, and Lateral Error to Positive=Left. | Previous models experienced the "death swerve" because steering and lateral error were inverted, and target offset (-0.238m) pushed the robot into the right-hand wall on narrow lanes. Standardizing to a consistent Right-Handed Frame directly solves this bias. | Yes | agent |
| D003 |  | physics | How to synchronize Isaac Lab physics with the real-world F1Tenth model. | Chassis mass override set to 3.342kg so total mass equals 4.092kg. effort_limit_sim reduced to 0.5. drive_scale remains at -40.0. | Properly matching the real-world 4.092kg mass and dialing down motor torque stops the robot from "kangaroo hopping" due to overpowered motors acting on mismatched mass. Maintaining drive_scale at -40.0 correctly moves the robot forward relative to the straightaway. | Yes | agent |
