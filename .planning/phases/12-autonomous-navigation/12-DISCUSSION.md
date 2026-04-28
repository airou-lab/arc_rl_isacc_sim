# Phase 12: Discussion Points

## 1. Global Map Dependency (Sim2Real Gap)
- **Topic**: The current V2I architecture assumes the robot reports its world-space (X, Y) to the JunctionModule.
- **Concern**: In the real world, "knowing exactly where you are" relative to a global map is difficult and often fails.
- **Open Question**: Can the robot communicate with the intersection using only **Local Relative Coordinates**? (e.g., "I am 2 meters from your Stop Line").
- **Goal**: Minimize the need for a "Global Reference Map" on the robot side to increase Sim2Real robustness.

## 2. PlanarPath Validation
- **Topic**: Submission of the 5-waypoint path to the junction.
- **Question**: Should the junction *correct* the path, or just *approve/deny* it?
