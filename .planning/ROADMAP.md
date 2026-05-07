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

- [x] **Phase 11: Intersection Navigation** - Retrained policy at 1x and implemented alignment-aware intersection gate permeability.
- [ ] **Phase 12: Autonomous Navigation** - Implement turning missions and proximity-based RoadGraph triggers for intersection decision-making.
- [ ] **Phase 13: Live Policy GUI** - Develop a standalone dashboard to stream telemetry, vision debugging, and policy transparency.
- [x] **Phase 14: Multi-Agent Environment Refactor** - Physically scaled USD geometry to 1x and refreshed boundary cache.

## Milestone 4: Multi-Agent Urban Coordination (v3.0) - PLANNED
*Focus: Smart intersections, multi-car coordination, and multi-agent RL (MARL).*

- [ ] **Phase 14-02: Multi-Agent Environment Refactor** - Adapt `ManagerBasedRLEnv` to handle multiple robot instances with independent observation/action streams.
- [ ] **Phase 15: Smart Intersection Infrastructure** - Implement intersection controllers (V2I/V2V) and dynamic traffic signaling in the USD.
- [ ] **Phase 16: Coordination & Yielding Logic** - Develop social-norm based navigation (e.g., yielding at stop signs, 4-way stop logic) between Two Cars.
- [ ] **Phase 17: MARL Training & Evaluation** - Train collaborative policies (MAPPO/IPPO) for high-density intersection throughput.

## Milestone 5: Custom Hardware & Sim2Real (v4.0) - PLANNED
*Focus: Transitioning from generic F1Tenth platforms to custom ARCPro hardware and real-world deployment.*

- [ ] **Phase 18: Custom ARCPro Robot Remodel** - Replace F1Tenth assets with a full-fidelity custom ARCPro robot model. Calibrate true-to-life physics (Torque, Mass, Friction).
- [ ] **Phase 19: Sim2Real Deployment** - Implement the self-driving policy on physical hardware with real-world perception bridges.

---
*Roadmap updated: 2026-04-20*
