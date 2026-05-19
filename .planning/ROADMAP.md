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

- [x] **Phase 11: Intersection Navigation** - Retrained policy at 1x and implemented alignment-aware intersection gate permeability.
- [x] **Phase 12: Autonomous Navigation** - Implement turning missions and proximity-based RoadGraph triggers for intersection decision-making.
- [x] **Phase 13: Live Policy GUI** - Develop a standalone dashboard to stream telemetry, vision debugging, and policy transparency.

## Milestone 3: HD Perception & Production Hardening (v2.5) - ACTIVE
*Focus: High-resolution vision, VRAM-optimized backbones, and production-scale training.*

- [x] **Phase 14: Procedural Multi-Agent Scaffolding** - Physically scaled USD geometry to 1x and established production training baseline for multi-agent support.
- [x] **Phase 15: HD Vision ResNet** - Research into ResNet-18 vs Custom CNN and uint8 optimization.
- [x] **Phase 15: Mastery Curriculum** - Single-Agent Lane-Following Mastery. Achieve full-lap capability (6000+ steps) using a dynamic boundary curriculum and memory-optimized HD vision.
- [ ] **Phase 16: MARL Transition** - **ACTIVE** - Refactor singletons to vectorized managers and establish multi-agent scene config.
- [ ] **Phase 17: Smart Intersection Infrastructure** - Implement intersection controllers (V2I/V2V) and dynamic traffic signaling in the USD.
- [ ] **Phase 18: Coordination & Yielding Logic** - Develop social-norm based navigation (e.g., yielding at stop signs, 4-way stop logic) between Two Cars.
- [ ] **Phase 19: MARL Training & Evaluation** - Train collaborative policies (MAPPO/IPPO) for high-density intersection throughput.

## Milestone 5: Custom Hardware & Sim2Real (v4.0) - PLANNED
*Focus: Transitioning from generic F1Tenth platforms to custom ARCPro hardware and real-world deployment.*

- [ ] **Phase 20: Custom ARCPro Robot Remodel** - Replace F1Tenth assets with a full-fidelity custom ARCPro robot model. Calibrate true-to-life physics (Torque, Mass, Friction).
- [ ] **Phase 21: Sim2Real Deployment** - Implement the self-driving policy on physical hardware with real-world perception bridges.

---
*Roadmap updated: 2026-05-11*
