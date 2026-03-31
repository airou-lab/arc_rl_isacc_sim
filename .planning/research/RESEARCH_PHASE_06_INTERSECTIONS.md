# Research: Smart Intersection & 3D Physics Hardening (Phase 5/6)

**Updated:** 2026-03-31
**Domain:** Isaac Sim, ROS 2, Asset Optimization, True Physics
**Confidence:** HIGH

---

## 1. Breakthroughs & Discoveries (March 2026 - Asset Verification)

During the investigation into why the "Original" USD map was failing at 1.0x metric scale, we identified critical conflicts between raw visual geometry and PhysX requirements.

### The 3D Waypoint Revelation
We discovered that `track_centerline.npy` contains **3D coordinates** with a ~2.5m Z-range.
*   **Insight**: The ARCPro navigation system is natively 3D.
*   **Correction**: We must align the track at world `(0,0,0)` if we want the 3D waypoints to match the 3D road surface. Offsetting the track globally (e.g., -1.25m) causes a physical/navigation mismatch.

### Physical "Baking" & Hardening Workflow
We developed a workflow to turn raw OSM exports into simulation-ready assets:
1.  **Flatten()**: Must be used to resolve all USD references into local geometry. This allows direct modification of meshes.
2.  **convexDecomposition**: Required for all drivable meshes to ensure small wheels do not "tunnel" through 2D surfaces.
3.  **Visual-Safe Binding**: When binding physics materials (Friction/Restitution), the `materialPurpose="physics"` parameter must be used.
    *   *Failure Mode*: Binding without a purpose overwrites the visual textures with a default white material.

### F1Tenth Traction Physics
*   **Mass**: The F1Tenth model was discovered to have `0kg` mass. In Isaac Sim, this results in zero downforce and zero traction.
*   **Fix**: Applied `3.5kg` to the chassis and `0.1kg` per wheel.
*   **Detection Buffer**: Added a `contactOffset` of `0.02m` to the robot wheels to improve collision detection against thin meshes.

### Isaac Sim UI Conflicts
*   **Manipulation Conflict**: Identified that `omni.kit.manipulator.selector` can crash if a script is rapidly updating the robot's pose while the user has the robot selected in the viewport.
*   **Workaround**: Deselect all objects in the Isaac Sim GUI before running automated verification scripts.

---

## 2. Legacy Phase 2: Smart Intersection Research

### Summary (2024-07-29)
The core task was adding a functional traffic light to an Isaac Sim scene and controlling it using ROS 2. The standard approach is to use pre-built assets (e.g., "Rivermark" traffic light) and control them via USD Variants.

### Pattern: ROS 2 Control of USD Variants
A ROS 2 node subscribes to a topic and uses the Isaac Sim Python API to switch the active USD Variant (Red, Yellow, Green).

**Example Node Logic:**
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import omni.usd

class TrafficLightController(Node):
    def listener_callback(self, msg):
        state = msg.data.capitalize() # "red" -> "Red"
        prim = self.stage.GetPrimAtPath(self.prim_path)
        variant_sets = prim.GetVariantSets()
        if variant_sets.HasVariantSet('state'):
            variant_sets.GetVariantSet('state').SetVariantSelection(state)
```

### Don't Hand-Roll
| Problem | Use Instead | Why |
| :--- | :--- | :--- |
| Traffic Light Model | "Rivermark" Asset | Has correct hierarchy and USD Variants built-in. |
| State Machine Logic | ROS 2 Publisher | Decouples control logic from simulation. |

---

## 3. Invisible Barriers (Tile Junctions)
Even with hardening, "raw" tile-based maps exhibit micro-cliffs at junctions.
*   **Lesson**: For high-speed RL training, a manually smoothed single-mesh track (like `no_graph_sim_final.usd`) is mandatory to avoid physical jitter and getting stuck.
*   **Status**: `original_production.usd` is technically functional but "bumpy."

## Sources
- **Project File Analysis (2026)**: Deep track audit revealed 1.8m variance across tile heights.
- **Numpy Analysis (2026)**: Waypoint inspection confirmed 3D navigation intent.
- **Web Search (2024)**: `Isaac Sim traffic light tutorial` for USD Variant control logic.
