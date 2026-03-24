# Phase 2: Smart Intersection Simulation - Research

**Researched:** 2024-07-29
**Domain:** Isaac Sim, ROS 2, Traffic Simulation
**Confidence:** HIGH

## Summary

This research phase focused on integrating a smart intersection into the existing F1Tenth project, which uses NVIDIA Isaac Sim and ROS 2. The project's simulation environment is built on Isaac Sim, not Gazebo as initially assumed. The core task is to add a functional traffic light to an Isaac Sim scene and control it using ROS 2.

The standard approach is to use pre-built assets from the Isaac Sim content library, specifically the "Rivermark" traffic light asset. This asset includes defined "USD Variants" for Red, Yellow, and Green states, which is the recommended control mechanism. Control will be implemented via a new ROS 2 node that subscribes to a topic for state changes and uses the Isaac Sim Python API to update the asset's variant in the simulation.

**Primary recommendation:** Use the existing "Rivermark" traffic light asset from the Isaac Sim asset library. Control its state by switching its USD Variants via a dedicated ROS 2 node that listens on a `std_msgs/String` topic.

## Standard Stack

### Core
| Component         | Version/Details          | Purpose                                        | Why Standard                                    |
|-------------------|--------------------------|------------------------------------------------|-------------------------------------------------|
| NVIDIA Isaac Sim  | 2022.2.1 or newer        | High-fidelity robotics simulation environment. | The project is already built on Isaac Sim.      |
| ROS 2             | Humble (assumed)         | Middleware for robotic communication.          | The project's policy and control code uses ROS 2. |
| USD               | -                        | Scene description format for Isaac Sim.        | Native format for all Isaac Sim assets.         |
| Python            | 3.7+                     | Scripting and controlling the simulation.      | Primary language for Isaac Sim API and ROS 2.   |

### Supporting
| Library/Asset          | Source                           | Purpose                                             | When to Use                                             |
|------------------------|----------------------------------|-----------------------------------------------------|---------------------------------------------------------|
| Rivermark Traffic Light| Isaac Sim Asset Library          | Pre-built traffic light model with defined states.  | For creating the smart intersection in the simulation.  |
| `rclpy`                | ROS 2                            | Python client library for ROS 2.                    | To create the control node for the traffic light.       |
| `omni.usd` / `pxr.Usd` | Isaac Sim Python API             | To interact with and manipulate the USD stage (scene).| To change the state of the traffic light in the sim.    |

**Installation:**
No new packages are required. The necessary Python libraries (`rclpy`, `omni.usd`) are included with the standard Isaac Sim and ROS 2 installation. The traffic light asset must be sourced from the Isaac Sim content browser.

## Architecture Patterns

### Recommended Project Structure
The new control node should be added to the existing ROS 2 package structure. A likely location is within the `arc_rl_isacc_policy` package, or a new package for world simulation objects.

```
arc_rl_isacc_policy/
├── agent/
├── ...
└── simulation_controllers/   # New folder for world controllers
    ├── __init__.py
    └── traffic_light_controller.py  # New node
```

### Pattern 1: ROS 2 Control of USD Variants
**What:** A ROS 2 node subscribes to a topic to receive state commands. When a message is received, it uses the Isaac Sim Python API to find the traffic light prim in the simulation stage and change its active USD Variant.

**When to use:** This is the primary pattern for controlling the state of the smart traffic light.

**Example:**
```python
# Source: Adapted from NVIDIA Isaac Sim documentation
# File: arc_rl_isacc_policy/simulation_controllers/traffic_light_controller.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from pxr import Usd
import omni.usd

class TrafficLightController(Node):
    def __init__(self, prim_path="/World/TrafficLight"):
        super().__init__('traffic_light_controller')
        self.prim_path = prim_path
        self.stage = omni.usd.get_context().get_stage()
        self.subscription = self.create_subscription(
            String,
            'traffic_light_state',
            self.listener_callback,
            10)
        self.get_logger().info(f"Traffic light controller started for prim: {self.prim_path}")

    def listener_callback(self, msg):
        state = msg.data.capitalize() # "red" -> "Red"
        self.get_logger().info(f'Setting traffic light to: "{state}"')
        
        prim = self.stage.GetPrimAtPath(self.prim_path)
        if not prim:
            self.get_logger().error(f"Prim not found at path: {self.prim_path}")
            return

        variant_sets = prim.GetVariantSets()
        if not variant_sets.HasVariantSet('state'):
            self.get_logger().error(f"No 'state' variant set found on prim: {self.prim_path}")
            return
            
        state_variant_set = variant_sets.GetVariantSet('state')
        
        if state in state_variant_set.GetVariantNames():
            state_variant_set.SetVariantSelection(state)
        else:
            self.get_logger().warn(f"Variant '{state}' not found in variant set.")

# Example of how to run this node would be from within Isaac Sim's script editor
# or as a standalone python application with Isaac Sim.
```

### Anti-Patterns to Avoid
- **Directly Manipulating Light Intensity:** Avoid controlling individual light bulb meshes' `intensity` or `visibility`. Using USD Variants is cleaner, less error-prone, and encapsulates the asset's state correctly.
- **Hardcoding Prim Paths:** The path to the traffic light prim (`/World/TrafficLight`) should be a configurable parameter, not a hardcoded string.

## Don't Hand-Roll

| Problem                 | Don't Build                               | Use Instead                           | Why                                                                   |
|-------------------------|-------------------------------------------|---------------------------------------|-----------------------------------------------------------------------|
| Traffic Light Model     | A custom traffic light from basic shapes. | The "Rivermark" asset from Isaac Sim. | The pre-built asset has correct materials, hierarchy, and USD Variants. |
| State Machine Logic     | A complex state machine inside the sim.   | A simple ROS 2 topic and publisher.   | Decouples the control logic from the simulation environment.          |

**Key insight:** The Isaac Sim ecosystem is designed around reusing high-quality, pre-built USD assets. The primary development effort should be in scripting and controlling these assets, not creating them from scratch.

## Common Pitfalls

### Pitfall 1: Prim Not Found
**What goes wrong:** The Python script fails because it cannot find the traffic light prim in the USD stage.
**Why it happens:** The prim path is incorrect, the model hasn't been loaded into the stage yet, or there's a typo.
**How to avoid:** Ensure the prim path passed to the controller node matches the path in the Isaac Sim stage hierarchy. Add checks to ensure the prim exists before trying to access it.
**Warning signs:** `Error: Prim not found at path: /World/TrafficLight` in the console logs.

### Pitfall 2: Simulation Not Stepping
**What goes wrong:** The visual state of the traffic light does not update in the viewport even though the script runs.
**Why it happens:** The Isaac Sim `world.step()` function is not being called in a loop. The simulation only updates its state when `step()` is called.
**How to avoid:** Ensure your main simulation script or the Isaac Sim editor is running and calling `world.step()` continuously.
**Warning signs:** ROS messages are received, logs show the variant is being set, but nothing changes visually.

## Code Examples

### Setting a USD Variant
This is the core operation for changing the traffic light state.

```typescript
// Source: NVIDIA Isaac Sim Documentation
import omni.usd

def set_traffic_light_state(prim_path: str, variant_name: str):
    """
    Sets the 'state' variant on a specified prim.
    
    Args:
        prim_path (str): The full path to the prim in the USD stage.
        variant_name (str): The name of the variant to set (e.g., "Red", "Green").
    """
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    
    if prim:
        variant_sets = prim.GetVariantSets()
        if variant_sets.HasVariantSet('state'):
            state_variant_set = variant_sets.GetVariantSet('state')
            state_variant_set.SetVariantSelection(variant_name)
        else:
            print(f"Warning: No 'state' variant set on prim {prim_path}")
    else:
        print(f"Error: Prim not found at {prim_path}")

# Example Usage:
# set_traffic_light_state("/World/MyTrafficLight", "Red")
```

## Open Questions

1.  **Exact Asset Path:** The exact path to the Rivermark traffic light asset in the content browser may vary depending on the Isaac Sim version. This will need to be confirmed when implementing.
2.  **Intersection Model:** This research focuses on the traffic light itself. The intersection road model is assumed to exist in the `arcpro_RL_open_street_sim.usd` file. If not, a separate effort will be needed to create or import an intersection scene.

## Sources

### Primary (HIGH confidence)
- **Web Search:** `Isaac Sim traffic light tutorial` - Provided the core concepts of using USD Variants and the Python API for control.
- **Project File Analysis:** `ls -R` and `grep` on the project directory confirmed the use of Isaac Sim and the absence of existing traffic light models.
