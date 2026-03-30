---
status: in_progress
phase: 05-training-policy-development
source: [05-02-PLAN.md]
---

# Phase 05: Hierarchical Policy Integration UAT

## Mandatory Tests

### 1. 12-Float Telemetry Protocol
**Status:** `pending`
**Requirement:** `REQ-SIM-METRIC`
**Expected:** The observation vector (indices 0-11) is fully populated with realistic values (turn_token, go_signal, speed, yaw_rate, etc.).
**Verification:** Run `verify_metric.py` and inspect logs.

### 2. Gaussian 'Hybrid Racer' Rewards
**Status:** `pending`
**Requirement:** `REQ-REWARDS-HYBRID`
**Expected:** Rewards follow a Gaussian distribution centered on the lane (higher near center, lower near edge).
**Verification:** Training telemetry logs showing reward/lateral_error correlation.

### 3. Lane-Aligned Spawning
**Status:** `pending`
**Requirement:** `REQ-SPAWN-LANE`
**Expected:** Robots reset to random track locations with randomized lateral (0.15m) and heading (5 deg) offsets.
**Verification:** Run `verify_spawn.py` and visually confirm multiple resets.

### 4. Hierarchical Inference Loop
**Status:** `pending`
**Requirement:** `REQ-NAV-HIERARCHY`
**Expected:** The `AgentNode` state machine correctly cycles between worker steps and policy inference.
**Verification:** `verify_policy.py` showing successful lane following and intersection detection.

## Human Verification Required

### 1. Visual Scale Confirmation (P0)
**Test:** Launch simulation using `verify_policy.py`.
**Expected:** F1Tenth robot (~33cm) and road lanes (~3.5m) appear proportionally correct.

### 2. Physical Stability Check (P0)
**Test:** Observe robot spawn/drop behavior.
**Expected:** Robot drops 0.5m, settles at Z=0 without clipping or jitters.

### 3. Metric Telemetry Audit (P1)
**Test:** Run `verify_metric.py` and observe logs.
**Expected:** speed < 5m/s, lat_err < 1.0m.

### 4. Baseline Policy Re-Verification (P2)
**Test:** Run `verify_policy.py` with ResNet18 model.
**Expected:** Old model still functions at 1.0x metric scale.
