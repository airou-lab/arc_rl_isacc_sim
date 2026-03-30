# Roadmap: ARCPro RL Isaac Sim Migration

## Overview
This roadmap outlines the migration of the ARCPro RL system to NVIDIA Isaac Lab and the integration of the Hierarchical Path Planning Policy.

## Phases

- [x] **Phase 1: Environment Foundation** - Established stable USD assets and connectivity.
- [x] **Phase 2: Isaac Lab Migration** - Refactored to vectorized `ManagerBasedRLEnv`.
- [x] **Phase 3: Infrastructure & CI/CD** - Implemented automated testing and documentation.
- [x] **Phase 4: Robot Refinement** - Applied 20x scaling and corrected spawn stability.
- [x] **Phase 7: Revert to True Physics Mode** - Reverted to 1.0x metric scaling for robot and track, grounding the simulation at Z=0 and removing workarounds.
- [ ] **Phase 5: Policy Integration** - Reorganize repository and hook up Hierarchical Policy architecture.
- [ ] **Phase 6: Intersection Navigation** - Implement graph-based route planning.

## Progress

| Phase | Plans Complete | Status | Completed |
|-------|----------------|--------|-----------|
| 1. Environment Foundation | 1/1 | COMPLETE | 2026-03-20 |
| 2. Isaac Lab Migration | 1/1 | COMPLETE | 2026-03-21 |
| 3. Infrastructure | 1/1 | COMPLETE | 2026-03-22 |
| 4. Robot Refinement | 1/1 | COMPLETE | 2026-03-23 |
| 5. Policy Integration | 0/1 | In Progress | - |
| 6. Intersection Nav | 0/3 | Planned | - |
| 7. Revert to True Physics Mode | 1/1 | COMPLETE | 2026-03-30 |

---
*Roadmap updated: 2026-03-30*
