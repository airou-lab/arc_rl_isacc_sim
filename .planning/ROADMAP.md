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

## Milestone 2: Autonomous Urban Navigation (v2.0) - COMPLETE
*Focus: Graph-based routing, intersection logic, and multi-agent coordination.*

- [x] **Phase 11:** 11-intersection-navigation - Retrained policy at 1x and implemented alignment-aware intersection gate permeability.
- [x] **Phase 12:** 12-autonomous-navigation - Implement turning missions and proximity-based RoadGraph triggers for intersection decision-making.
- [x] **Phase 13:** 13-live-policy-gui - Develop a standalone dashboard to stream telemetry, vision debugging, and policy transparency.

## Milestone 3: HD Perception & Production Hardening (v2.5) - ACTIVE
*Focus: High-resolution vision, VRAM-optimized backbones, and production-scale training.*

- [x] **Phase 14:** 14-01-procedural-multi-agent-scaffolding - Physically scaled USD geometry to 1x and established production training baseline for multi-agent support.
- [ ] **Phase 15:** 15-mastery-curriculum - **ACTIVE** - Single-Agent Lane-Following Mastery. Achieve full-lap capability (6000+ steps) using a dynamic boundary curriculum and memory-optimized HD vision.
    **Plans:** 2 plans
    - [ ] 15-01-PLAN.md — Dynamic Boundary Infrastructure & Precision Reward Tuning.
    - [ ] 15-02-PLAN.md — uint8 Memory Optimization & Curriculum Monitoring.

## Milestone 4: Multi-Agent Urban Coordination (v3.0) - PLANNED
*Focus: Smart intersections, multi-car coordination, and multi-agent RL (MARL).*

- [ ] **Phase 16: Multi-Agent Urban Coordination (MARL transition)** - **PLANNED** - Refactor singletons to vectorized managers and establish multi-agent scene config.
- [ ] **Phase 17: Smart Intersection Infrastructure** - Implement intersection controllers (V2I/V2V) and dynamic traffic signaling in the USD.
- [ ] **Phase 18: Coordination & Yielding Logic** - Develop social-norm based navigation (e.g., yielding at stop signs, 4-way stop logic) between Two Cars.
- [ ] **Phase 19: MARL Training & Evaluation** - Train collaborative policies (MAPPO/IPPO) for high-density intersection throughput.

## Milestone 5: Custom Hardware & Sim2Real (v4.0) - PLANNED
*Focus: Transitioning from generic F1Tenth platforms to custom ARCPro hardware and real-world deployment.*

- [ ] **Phase 20: Custom ARCPro Robot Remodel** - Replace F1Tenth assets with a full-fidelity custom ARCPro robot model. Calibrate true-to-life physics (Torque, Mass, Friction).
- [ ] **Phase 21: Sim2Real Deployment** - Implement the self-driving policy on physical hardware with real-world perception bridges.

---
*Roadmap updated: 2026-05-11*
