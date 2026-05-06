# Project: ARCPro RL Isaac Lab Migration

## What This Is
A high-fidelity reinforcement learning environment for autonomous mobile robots, built on NVIDIA Isaac Lab. It facilitates the development and validation of self-driving policies in complex, true-to-scale urban environments.

## Core Value
To provide a physically accurate, high-performance simulation bridge that minimizes sim-to-real gap for ARCPro autonomous systems through 1.0x metric scaling, realistic multi-body physics, and graph-based navigation.

## Goals
- **v1.2 (Foundation):** Stabilized 1.0x metric scaling and hierarchical policy stack.
- **v2.0 (Navigation):** Graph-based intersection crossing and random mission generation.
- **v2.5 (Visibility):** Real-time telemetry visualization and policy diagnostics.
- **v3.0 (Multi-Agent):** Collaborative multi-robot coordination and smart infrastructure.

## Context
- **Framework:** NVIDIA Isaac Lab / Isaac Sim 5.x.
- **Target Platform:** F1Tenth (Scaling to custom ARCPro hardware).
- **Control Rate:** 20Hz (DT=0.05s).
- **Physics Solver:** TGS (16 iterations).

## Technical Requirements
- **Hardware:** NVIDIA RTX GPU (8GB+ VRAM recommended).
- **Software:** Ubuntu 22.04+, Isaac Lab, Stable Baselines 3 Contrib.
- **Scale:** Native 1.0x metric (1 unit = 1 meter).
