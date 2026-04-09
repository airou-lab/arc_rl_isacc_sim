# Roadmap: ARCPro RL Isaac Sim Migration

## Overview
This roadmap outlines the migration of the ARCPro RL system to NVIDIA Isaac Lab.

## Milestone 1: Physical Fidelity & Policy Foundation (v1.2)
*Focus: 1.0x metric stabilization, hierarchical policy stack, and true physics restoration.*

- [x] **Phase 1: Environment Foundation** - Established stable USD assets and connectivity.
- [x] **Phase 2: Isaac Lab Migration** - Refactored to vectorized `ManagerBasedRLEnv`.
- [x] **Phase 3: Infrastructure & CI/CD** - Implemented automated testing and documentation.
- [x] **Phase 4: Robot Refinement** - Applied 20x scaling and corrected spawn stability.
- [x] **Phase 5: Policy Integration** - Integrated ResNet18 and 12-float telemetry.
- [x] **Phase 6: Physics Restoration** - Reverted to 1.0x metric scaling with 20kg mass.
- [x] **Phase 7: Gymnasium & Environment Hygiene** - Modernized API and cleaned up terminal noise.
- [x] **Phase 8: Physics Fidelity Restoration** - High-fidelity physics parameters (damping, stiffness).
- [ ] **Phase 9: Training Loop Stabilization** - (ACTIVE) Finalize reset logic and torque verification.

## Milestone 2: Autonomous Urban Navigation (v2.0)
*Focus: Graph-based routing, intersection logic, and multi-agent coordination.*

- [ ] **Phase 10: Intersection Navigation** - Implement graph-based route planning.

## Progress

| Phase | Milestone | Status | Completed |
|-------|-----------|--------|-----------|
| 1, 2, 3, 4 | M1 | COMPLETE | 2026-03-30 |
| 5. Policy Integration | M1 | COMPLETE | 2026-04-02 |
| 6, 7, 8. Physics & Hygiene | M1 | COMPLETE | 2026-04-05 |
| 9. Training Stability | M1 | COMPLETE | 2026-04-09 |
| 10. Intersection Nav | M2 | Planned | - |

---
*Roadmap updated: 2026-04-09*
