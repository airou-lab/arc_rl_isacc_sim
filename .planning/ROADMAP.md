# Roadmap: ARCPro RL Isaac Sim Migration

## Overview
This roadmap outlines the migration of the ARCPro RL system to NVIDIA Isaac Lab.

## Milestone 1: Physical Fidelity & Policy Foundation (v1.2)
*Focus: 1.0x metric stabilization, hierarchical policy stack, and true physics restoration.*

- [x] **Phase 1: Environment Foundation** - Established stable USD assets and connectivity.
- [x] **Phase 2: Isaac Lab Migration** - Refactored to vectorized `ManagerBasedRLEnv`.
- [x] **Phase 3: Infrastructure & CI/CD** - Implemented automated testing and documentation.
- [x] **Phase 4: Robot Refinement** - Applied 20x scaling and corrected spawn stability.
- [x] **Phase 7: Revert to True Physics Mode** - Reverted to 1.0x metric scaling.
- [ ] **Phase 5: Policy Integration** - Implement Hierarchical Policy and 12-float telemetry. (CURRENT FOCUS)
  - [ ] 05-01-PLAN.md — Integration of ResNet18 road-following policy. (PENDING USER APPROVAL / NEEDS TESTING)
  - [~] 05-01-ASSET-VERIFY-PLAN.md — (STALE) Verification of USD assets; needs revisit for "Invisible Barriers".
  - [ ] 05-02-PLAN.md — Implementation of Hierarchical Policy and 12-float telemetry. (WAITING USER CONFIRMATION)
- [ ] **Phase 8: F1Tenth Physics Fidelity Restoration** - Restore damping/stiffness and high-fidelity parameters.

## Milestone 2: Autonomous Urban Navigation (v2.0)
*Focus: Graph-based routing, intersection logic, and multi-agent coordination.*

- [ ] **Phase 6: Intersection Navigation** - Implement graph-based route planning.

## Progress

| Phase | Milestone | Status | Completed |
|-------|-----------|--------|-----------|
| 1, 2, 3, 4, 7 | M1 | COMPLETE | 2026-03-30 |
| 5. Policy Integration | M1 | In Progress | - |
| 8. Physics Restoration | M1 | Planned | - |
| 6. Intersection Nav | M2 | Planned | - |

---
*Roadmap updated: 2026-03-31*
