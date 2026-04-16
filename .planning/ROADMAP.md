# Roadmap: ARCPro RL Isaac Sim Migration

## Overview
This roadmap outlines the migration of the ARCPro RL system to NVIDIA Isaac Lab.

## Milestone 1: Physical Fidelity & Policy Foundation (v1.2) - COMPLETE
*Focus: 1.0x metric stabilization, hierarchical policy stack, and true physics restoration.*

- [x] **Phase 1: Environment Foundation** - Established stable USD assets and connectivity.
- [x] **Phase 2: Isaac Lab Migration** - Refactored to vectorized `ManagerBasedRLEnv`.
- [x] **Phase 3: Infrastructure & CI/CD** - Implemented automated testing and documentation.
- [x] **Phase 4: Robot Refinement** - Applied 20x scaling and corrected spawn stability.
- [x] **Phase 5: Policy Integration** - Integrated ResNet18 and 12-float telemetry.
- [x] **Phase 6: Physics Restoration** - Reverted to 1.0x metric scaling with 20kg mass.
- [x] **Phase 7: Gymnasium & Environment Hygiene** - Modernized API and cleaned up terminal noise.
- [x] **Phase 8: Physics Fidelity Restoration** - High-fidelity physics parameters (damping, stiffness).
- [x] **Phase 9: Training Loop Stabilization** - Stabilized 8x physics, implemented 2D action space, and confirmed PPO convergence.
- [x] **Phase 10: Asset Downscaling** - Reverted to 1.0x metric scale, isolated roads, and removed terrain clutter.

## Milestone 2: Autonomous Urban Navigation (v2.0) - ACTIVE
*Focus: Graph-based routing, intersection logic, and multi-agent coordination.*

- [/] **Phase 11: Retraining & Intersection logic** - (ACTIVE) Retrain policy at 1x and implement graph-based route planning.
- [ ] **Phase 12: Hierarchical Policy Migration** - Port the `Worker-Driver` architecture from the sibling folder and refactor for the physics.
- [x] **Phase 13: Environment Scale Alignment** - (COMPLETED via Phase 10) Scaled USD road assets to match the robot scale.

## Progress

| Phase | Milestone | Status | Completed |
|-------|-----------|--------|-----------|
| 1-4   | M1 | COMPLETE | 2026-03-30 |
| 5     | M1 | COMPLETE | 2026-04-02 |
| 6-8   | M1 | COMPLETE | 2026-04-05 |
| 9     | M1 | COMPLETE | 2026-04-11 |
| 10    | M1 | COMPLETE | 2026-04-12 |
| 13    | M2 | COMPLETE | 2026-04-12 |
| 11    | M2 | ACTIVE   | - |
| 12    | M2 | Planned  | - |
| 13    | M3 | Planned  | - |
| 14-17 | M4 | Planned  | - |

## Milestone 3: Real-Time Visualization & Diagnostics (v2.5) - PLANNED
*Focus: Live telemetry, vision debugging, and policy transparency.*

- [ ] **Phase 13: Live Policy GUI** - Develop a standalone dashboard to stream the robot's camera feed and real-time PPO action distributions (steering/throttle) alongside the simulation.

## Milestone 4: Multi-Agent Urban Coordination (v3.0) - PLANNED
*Focus: Smart intersections, multi-car coordination, and multi-agent RL (MARL).*

- [ ] **Phase 14: Multi-Agent Environment Refactor** - Adapt `ManagerBasedRLEnv` to handle multiple robot instances with independent observation/action streams.
- [ ] **Phase 15: Smart Intersection Infrastructure** - Implement intersection controllers (V2I/V2V) and dynamic traffic signaling in the USD.
- [ ] **Phase 16: Coordination & Yielding Logic** - Develop social-norm based navigation (e.g., yielding at stop signs, 4-way stop logic) between Two Cars.
- [ ] **Phase 17: MARL Training & Evaluation** - Train collaborative policies (MAPPO/IPPO) for high-density intersection throughput.

---
*Roadmap updated: 2026-04-15*
