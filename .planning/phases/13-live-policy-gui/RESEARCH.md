# Phase 13: Live Policy GUI - Research

**Researched:** 2026-04-15
**Domain:** Real-time Robotics Telemetry & Visualization
**Confidence:** HIGH

## Summary

This research identifies the optimal architecture for a real-time policy dashboard in Isaac Lab. The primary challenge is maintaining the high simulation frame rate (~115 FPS) while streaming high-bandwidth RGB camera data and policy distributions without inducing simulation jitter.

**Primary recommendation:** Use a **Sidecar GUI process** communicating via **Shared Memory (`multiprocessing.shared_memory`)**, implemented with **OpenCV** for low-latency rendering and **PyQt5** for layout management. This avoids the simulation-blocking nature of "In-process" GUIs and the high latency of web-based frameworks like Streamlit.

## Standard Stack

### Core
| Library | Version | Purpose | Why Standard |
|---------|---------|---------|--------------|
| OpenCV | 4.11.0 | Image processing/rendering | Low-latency, standard in Isaac Lab python. |
| SharedMemory | 3.8+ | Zero-copy data streaming | Fastest IPC for large image buffers. |
| PyQt5 | 5.15+ | Dashboard Layout/Widgets | Robust, multi-threaded, and high-performance. |

### Supporting
| Library | Version | Purpose | When to Use |
|---------|---------|---------|--------------|
| NumPy | 1.26.x | Array manipulation | High-performance matrix operations. |
| Stable Baselines 3 | 2.1+ | Policy distribution access | To visualize PPO action heatmaps. |
| Rerun.io | 0.31+ | Advanced visualization | Alternative to custom GUI for 3D/Multimodal data. |

### Alternatives Considered
| Instead of | Could Use | Tradeoff |
|------------|-----------|----------|
| OpenCV Sidecar | Streamlit | Streamlit is too slow for 115 FPS video; high latency. |
| OpenCV Sidecar | Gradio | Gradio is designed for demos, not high-frequency telemetry. |
| Sidecar | In-process (omni.ui) | In-process GUIs block the simulation thread, causing FPS drops. |

## Architecture Patterns

### Recommended Project Structure
```
arcproLab/
├── mdp/
│   └── visual_analytics.py   # Shared memory writer logic
└── scripts/
    └── policy_dashboard.py   # Sidecar reader & GUI process
```

### Pattern 1: Shared Memory Sidecar
**What:** The simulation process writes raw RGB frames and telemetry vectors to a pre-allocated `SharedMemory` block. A separate GUI process reads this block at its own refresh rate.
**When to use:** High-frequency video streaming (>30 FPS) from Isaac Sim.
**Example:**
```python
# Simulation Side (Producer)
from multiprocessing import shared_memory
shm = shared_memory.SharedMemory(name="arcpro_telemetry", create=True, size=1024*1024)
buffer = np.ndarray((480, 640, 3), dtype=np.uint8, buffer=shm.buf)
# Inside loop:
buffer[:] = camera_rgb_frame
```

### Anti-Patterns to Avoid
- **Blocking `cv2.imshow`:** Never call `cv2.waitKey()` with a long delay inside the main simulation loop; it will cap the simulation FPS to the wait time.
- **Web-sockets for Video:** Avoid streaming raw 640x480 RGB at 100+ FPS over standard WebSockets; serialization overhead will saturate a single CPU core.

## Don't Hand-Roll

| Problem | Don't Build | Use Instead | Why |
|---------|-------------|-------------|-----|
| UI Layout | Custom pixel-grid | PyQt5 / PySide6 | Handling window resizing and widget placement manually is error-prone. |
| Video Sync | Custom ring-buffer | SharedMemory | OS-level shared memory is faster and more reliable than manual pipes. |
| Action Distribution | Custom PDF plotter | SB3 Distribution heads | Policy distributions (Gaussian) have native `mean` and `std` access. |

## Common Pitfalls

### Pitfall 1: Shared Memory Leaks
**What goes wrong:** If the simulation crashes, the shared memory segment stays allocated in RAM.
**Why it happens:** Linux kernel persists `shm` until `unlink()` is called.
**How to avoid:** Use a `try...finally` block to call `shm.close()` and `shm.unlink()` on exit.

### Pitfall 2: Python Version Mismatch
**What goes wrong:** Isaac Lab uses Python 3.11/3.10; system ROS 2 or PyQt might use 3.12.
**Why it happens:** Incompatible C-extensions.
**How to avoid:** Run the GUI using the same virtual environment/interpreter as Isaac Lab if possible, or use a purely primitive communication protocol (SharedMemory/Sockets) that doesn't depend on complex library serialization (like Pickle).

## Code Examples

### Accessing PPO Action Distribution (Heatmap Data)
```python
# Source: Stable Baselines 3 API
# Extracting distribution for a given observation 'obs'
distribution = model.policy.get_distribution(obs)
mean = distribution.distribution.mean  # [Steering, Throttle]
std = distribution.distribution.stddev # [Steering, Throttle]

# Draw as a 2D Gaussian Heatmap in OpenCV
# Center = mean, Axes = std
```

### OpenCV Layout Assembly
```python
# Assemble dashboard from parts
canvas = np.zeros((720, 1280, 3), dtype=np.uint8)
canvas[0:480, 0:640] = camera_view
canvas[0:200, 640:840] = action_heatmap
# Add text overlays for telemetry
cv2.putText(canvas, f"LatErr: {lat_err:.3f}", (650, 250), ...)
```

## State of the Art

| Old Approach | Current Approach | When Changed | Impact |
|--------------|------------------|--------------|--------|
| `omni.ui` labels | Rerun / SharedMemory Sidecar | 2024 | Separates simulation physics from UI rendering. |
| Matplotlib animation | PyQtGraph / OpenCV | - | Real-time (>60Hz) vs Static/Batch plotting. |

## Open Questions

1. **Stochastic vs Deterministic:** During inference (`deterministic=True`), PPO doesn't strictly provide a "distribution" in the same way as training. Should the GUI show the training distribution or just the deterministic point?
   - **Recommendation:** Show the training distribution (mean/std) to visualize the policy's "confidence".

## Environment Availability

| Dependency | Required By | Available | Version | Fallback |
|------------|------------|-----------|---------|----------|
| OpenCV | Video Rendering | ✓ | 4.11.0 | — |
| PyQt5 | UI Layout | ✓ | 5.15.10 | Use raw OpenCV |
| SharedMemory| IPC | ✓ | — | Unix Domain Sockets |
| X11 Display | GUI | ✓ | — | Save to video file (headless) |

## Validation Architecture

### Test Framework
| Property | Value |
|----------|-------|
| Framework | pytest |
| Config file | arcproLab/policy_stack/pytest.ini |
| Quick run command | `pytest arcproLab/policy_stack/tests/` |

### Phase Requirements → Test Map
| Req ID | Behavior | Test Type | Automated Command | File Exists? |
|--------|----------|-----------|-------------------|-------------|
| REQ-GUI-FPS | Sidecar GUI maintains > 100 FPS | Performance | `python3 verify_sim_metric.sh` | ❌ Wave 0 |
| REQ-GUI-LAT | Latency from sim to GUI < 10ms | Integration | Custom latency test | ❌ Wave 0 |

## Sources

### Primary (HIGH confidence)
- [NVIDIA Isaac Lab Docs] - Simulation loop and camera sensor API.
- [Stable Baselines 3 Docs] - Policy distribution and `mean`/`std` extraction.
- [Python Multiprocessing Docs] - `shared_memory` usage and constraints.

### Secondary (MEDIUM confidence)
- [Rerun.io Blog] - Best practices for robotics visualization.

## Metadata

**Confidence breakdown:**
- Standard stack: HIGH - OpenCV/PyQt/SharedMemory are industry standard.
- Architecture: HIGH - Sidecar pattern is essential for high-frequency simulation.
- Pitfalls: MEDIUM - Shared memory cleanup is the main risk.

**Research date:** 2026-04-15
**Valid until:** 2026-05-15
