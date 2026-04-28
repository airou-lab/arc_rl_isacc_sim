# Phase 13: Live Policy GUI - Research

**Researched:** 2024-05
**Domain:** Python Telemetry Visualization & Robotics Dashboarding
**Confidence:** HIGH

## Summary

This phase pivots the inference visualization dashboard from a custom PyQt5/ZMQ architecture to an off-the-shelf Dockerized/web viewer. The goal is to stream 100FPS RGB images, scalar rewards, and action distributions during Isaac Sim evaluation (`verify_policy.py`) without blocking the physics loop.

**Primary recommendation:** Use **Rerun.io** over Foxglove. Rerun's native Python SDK (`rr.log`) and Apache Arrow/gRPC backend is perfectly optimized for high-frequency (100Hz+) streaming of raw NumPy arrays directly from the training/inference loop, requiring no custom bridges or ZeroMQ boilerplate.

## Standard Stack

### Core
| Library | Version | Purpose | Why Standard |
|---------|---------|---------|--------------|
| rerun-sdk | >=0.15.0 | Telemetry and visualization | Native Python SDK, built-in asynchronous micro-batching, easily streams 100 FPS raw tensors via gRPC without blocking Isaac Sim. |
| sb3-contrib | Current | RecurrentPPO policy handling | Required for extracting LSTM-based action distributions during inference. |
| opencv-python | Current | Image conversion | Used to format RGB tensors into plottable images if required before logging. |

### Alternatives Considered
| Instead of | Could Use | Tradeoff |
|------------|-----------|----------|
| Rerun.io | Foxglove | Foxglove is the industry standard for fleet ops and ROS. However, streaming 100fps raw RGB from a raw Python process requires building a custom WebSocket bridge (or using ROS/MCAP), whereas Rerun is native `import rerun as rr; rr.log()`. |
| PyQt5 + ZMQ | Rerun.io | PyQt5 requires hand-rolling threaded UI loops and IPC (ZMQ). Rerun manages background threading, serialization, and UI layouts out-of-the-box. |

## Architecture Patterns

### Recommended Integration Pattern
Use Rerun's background logging threaded architecture (`rr.spawn()` or `rr.serve()`) directly inside `verify_policy.py`.

**What:** Direct logging from the step loop.
**When to use:** During `verify_policy.py` inference when `--dashboard` is enabled.
**Example:**
```python
import rerun as rr

if args_cli.dashboard:
    # Spawns a native viewer or hosts a web viewer via rr.serve()
    rr.init("arcpro_policy_inference", spawn=True)

# Inside the simulation loop:
if args_cli.dashboard:
    rr.set_time_sequence("step", count)
    rr.log("robot/camera/rgb", rr.Image(img_np))
    rr.log("policy/action/mean", rr.Tensor(action_mean))
    # ... log rewards
```

## Don't Hand-Roll

| Problem | Don't Build | Use Instead | Why |
|---------|-------------|-------------|-----|
| IPC & Threading | ZeroMQ / Shared Memory + PyQt5 | Rerun.io | Rerun automatically batches logs on a background thread and sends them over gRPC to the viewer. Hand-rolling ZMQ pub/sub for 100Hz video is brittle and reinventing the wheel. |
| Time Syncing | Custom buffer dicts | Rerun Timelines | Rerun automatically aligns heterogeneous data (scalars, images, tensors) to a defined timeline (e.g., simulation step or time). |

## Common Pitfalls

### Pitfall 1: Saturating Network/CPU with Multi-Environment Rendering
**What goes wrong:** The dashboard lags or crashes when `num_envs > 1` (e.g. batch size of 64 or 1024).
**Why it happens:** Logging 1024 high-res RGB images and action distributions at 100Hz requires massive bandwidth.
**How to avoid:** Only log telemetry for `env_idx = 0`. Extract the 0th index from the batched tensors before calling `rr.log()`.

### Pitfall 2: Synchronous Blocking
**What goes wrong:** `verify_policy.py` framerate drops significantly when the dashboard is enabled.
**Why it happens:** Heavy tensor conversions or waiting for network ACKs.
**How to avoid:** Rerun batches asynchronously by default, but you must avoid expensive CPU tensor ops (e.g. nested loops) before logging.

### Pitfall 3: Not Handling Environment Resets
**What goes wrong:** The reward graphs draw continuous lines connecting the end of one episode to the start of the next, creating visual noise.
**Why it happens:** Timelines are strictly monotonic.
**How to avoid:** Use Rerun's `rr.log("...", rr.Clear(recursive=True))` when `dones[0]` is True to clear episodic graphs, or visually group logs by an `episode_id` timeline.

## Code Examples

### 1. Extracting Action Distribution from RecurrentPPO
Stable Baselines 3 uses PyTorch distributions under the hood. For `RecurrentPPO`, you must provide the observations, LSTM states, and episode starts.

```python
import torch
from stable_baselines3.common.utils import obs_as_tensor

# Convert obs to tensor on correct device
obs_tensor = obs_as_tensor(obs, model.device)

# Get the distribution without updating gradients
with torch.no_grad():
    distribution, _ = model.policy.get_distribution(
        obs_tensor, 
        lstm_states, 
        episode_starts
    )

# Extract Mean and Standard Deviation (for Continuous Gaussian actions)
action_mean = distribution.distribution.mean[0].cpu().numpy()
action_std = distribution.distribution.stddev[0].cpu().numpy()

# Log to dashboard
rr.log("policy/action_mean", rr.Tensor(action_mean))
rr.log("policy/action_std", rr.Tensor(action_std))
```

### 2. Extracting Live Reward Components from ManagerBasedRLEnv
Isaac Lab's `RewardManager` tracks individual reward terms. We can extract the current step values for the first environment.

```python
# Access the raw unwrapped environment from the SB3 VecEnv chain
raw_env = env.venv.venv.unwrapped
reward_manager = raw_env.reward_manager

# Iterate active terms for environment index 0
for term_name, term_value in reward_manager.get_active_iterable_terms(env_idx=0):
    val = term_value.item()
    rr.log(f"rewards/components/{term_name}", rr.Scalar(val))

# Log aggregate step reward
rr.log("rewards/aggregate", rr.Scalar(rewards[0]))
```

## State of the Art

| Old Approach | Current Approach | When Changed | Impact |
|--------------|------------------|--------------|--------|
| Custom PyQt5/ZMQ GUI | Rerun.io Native SDK | 2023-2024 | Drastically reduces boilerplate code. Rerun acts as a lightweight sidecar that ingests data asynchronously without custom IPC logic. |

## Open Questions

1. **Viewer Hosting (Dockerized Web Viewer vs Native App)**
   - What we know: The user requested a "Dockerized web dashboard". Rerun can serve a web application via `rr.serve(web_port=9090)`.
   - What's unclear: Does the user want a persistent Docker container running the visualizer, or is `verify_policy.py` dynamically spinning up the web server?
   - Recommendation: Use `rr.serve()` in the Python script. It automatically hosts the web viewer and WebSocket endpoint without needing a separate Dockerfile, satisfying the "web dashboard" requirement trivially.

## Environment Availability

| Dependency | Required By | Available | Version | Fallback |
|------------|------------|-----------|---------|----------|
| rerun-sdk | Telemetry Dashboard | ✗ | — | Pip install rerun-sdk |
| sb3-contrib | Action Distribution | ✓ | — | — |
| isaaclab | Reward Extraction | ✓ | — | — |

**Missing dependencies with no fallback:**
- `rerun-sdk` must be installed in the Isaac Sim python environment (`./isaaclab.sh -p -m pip install rerun-sdk`).

## Validation Architecture

### Test Framework
| Property | Value |
|----------|-------|
| Framework | pytest |
| Quick run command | `./isaaclab.sh -p -m pytest arcproLab/policy_stack/tests/ -v` |

### Phase Requirements → Test Map
| Req ID | Behavior | Test Type | Automated Command | File Exists? |
|--------|----------|-----------|-------------------|-------------|
| REQ-13-01 | Action dist extraction works without crashing | unit | `pytest ...::test_action_extraction` | ❌ Wave 0 |
| REQ-13-02 | Reward component extraction works | unit | `pytest ...::test_reward_extraction` | ❌ Wave 0 |

## Sources

### Primary (HIGH confidence)
- Rerun.io Official Docs - Python SDK setup, asynchronous micro-batching.
- Stable Baselines 3 & sb3-contrib source - `RecurrentActorCriticPolicy.get_distribution` signature requiring `lstm_states`.
- Isaac Lab GitHub & Docs - `RewardManager.get_active_iterable_terms` functionality.

### Secondary (MEDIUM confidence)
- Community discussions on Rerun vs Foxglove for local Python-driven high-FPS robotics visualization.

## Metadata

**Confidence breakdown:**
- Standard stack: HIGH - Rerun is explicitly built for this Python-first high-frequency use case.
- Architecture: HIGH - Verified SB3 RecurrentPPO distribution extraction and Isaac Lab reward manager internals.
- Pitfalls: HIGH - Batch size and reset handling are common issues in RL telemetry visualization.

**Research date:** 2024-05
**Valid until:** 2024-11
