# Todo: Integrate Reset Logic
Goal: Port robust waypoint snapping to the dev branch.

## Tasks
- [ ] Extract `reset_robot_to_lane` logic from `feat/waypoint-snapping`.
- [ ] Apply direct Z=0.05m snapping to `arcproLab/mdp/events.py`.
- [ ] Verify that the robot no longer falls back to Z=10.0.
