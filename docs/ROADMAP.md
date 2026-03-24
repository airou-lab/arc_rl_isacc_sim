# Master Roadmap: ARCPro RL System

## Phase 1: Environment & Robot Foundation (COMPLETE)
- [x] **1.1: Connectivity** - Verified ROS2 installation and bridge connectivity.
- [x] **1.2: Robot Integration** - Sourced and stabilized F1Tenth USD assets.
- [x] **1.3: Track Alignment** - Restored OpenStreetUSD geometry.

## Phase 2: Isaac Lab Migration (COMPLETE)
- [x] **2.1: Task Definition** - Refactor USD into Isaac Lab schema.
- [x] **2.2: Manager Configuration** - Implement Observation, Action, and Reward Managers.
- [x] **2.3: Vectorization** - Transition to multi-robot parallel training (128+ agents).

## Phase 3: Infrastructure & CI/CD (COMPLETE)
- [x] **3.1: Documentation Recovery** - Audit and restore missing context.
- [x] **3.2: Verification Loop** - Physical verification loop for environment fidelity.
- [x] **3.3: CI/CD Integration** - GitHub Actions workflow with unit tests.

## Phase 4: Robot Refinement & Verification (COMPLETE)
- [x] **4.1: Robot Scaling & Metrics** - Applied physical measurements (403mm L, 287mm W, 25cm wheelbase).
- [x] **4.2: Sensor Optimization** - Configured RGB-only camera pipeline (Realsense at 145/0/195).
- [x] **4.3: Free Camera Support** - Implemented viewer tracking for debugging.
- [x] **4.4: Clipping & Hardening** - Verified physics stability and drop-test stabilization.

## Phase 5: Training & Policy Development (IMMEDIATE PRIORITY)
- [ ] **5.1: Original Policy Integration** - Link SB3 policy with Isaac Lab environment.
- [ ] **5.2: Inference Verification** - Confirm autonomous lap completion.
- [ ] **5.3: Visual Analytics** - Isaac Lab compliant GUI/HUD for debugging.

## Phase 6: Intersection & Graph-Based Navigation (FUTURE)
- [ ] **6.1: Graph Track Manager** - Implement edge/node topology for road segments.
- [ ] **6.2: Navigation Commands** - Add "Straight/Left/Right" inputs to the policy.
- [ ] **6.3: Dynamic Path Rewards** - Reward adherence to high-level navigation goals.
