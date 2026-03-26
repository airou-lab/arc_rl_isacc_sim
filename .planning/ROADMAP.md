# Roadmap: ARCPro RL Isaac Sim Migration

## Overview
This roadmap outlines the journey of migrating the ARCPro RL system to NVIDIA Isaac Lab for high-performance training.

## Phases

- [x] **Phase 1: Environment & Robot Foundation** - Established stable USD assets and connectivity.
- [x] **Phase 2: Isaac Lab Migration** - Refactored to vectorized `ManagerBasedRLEnv`.
- [x] **Phase 3: Infrastructure & CI/CD** - Implemented automated testing and documentation.
- [x] **Phase 4: Robot Refinement** - Applied 20x scaling and 1000Hz frequency for stability.
- [x] **Phase 5: Policy Integration** - Integrated SB3 policy and verified autonomous driving.
- [ ] **Phase 6: Intersection & Graph-Based Navigation** - Implement edge/node topology for road segments and traffic light control.

## Progress

| Phase | Plans Complete | Status | Completed |
|-------|----------------|--------|-----------|
| 1. Environment Foundation | 1/1 | COMPLETE | 2026-03-20 |
| 2. Isaac Lab Migration | 1/1 | COMPLETE | 2026-03-21 |
| 3. Infrastructure | 1/1 | COMPLETE | 2026-03-22 |
| 4. Robot Refinement | 1/1 | COMPLETE | 2026-03-23 |
| 5. Policy Integration | 1/1 | COMPLETE | 2026-03-25 |
| 6. Graph Navigation | 0/3 | PLANNED | - |

---

### Phase 6: Intersection & Graph-Based Navigation
**Goal:** Navigate complex intersections with traffic light awareness using a graph-based road topology.
**Requirements:** REQ-NAV-GRAPH, REQ-NAV-TRANSITION, REQ-INT-LIGHT, REQ-INT-CTRL, REQ-INT-OBS, REQ-INT-REWARD

**Plans:**
- [ ] 06-01-PLAN.md — Implement RoadGraph and enhanced TrackManager.
- [ ] 06-02-PLAN.md — Integrate traffic light assets and ROS 2 controller.
- [ ] 06-03-PLAN.md — Update RL observations and rewards for intersection awareness.

---
*Roadmap updated: 2026-03-25*
