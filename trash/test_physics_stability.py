from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

from omni.isaac.core import World
from omni.isaac.core.articulations import ArticulationView
import numpy as np

def test_physics():
    print("[PhysicsTest] Initializing...")
    world = World(stage_units_in_meters=1.0)
    world.get_physics_context().set_gravity(-9.81)
    
    # Ground plane is mandatory for a drop test
    from omni.isaac.core.objects import VisualPlane
    world.scene.add_default_ground_plane()
    
    usd_path = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/f1tenth_trainer/assets/F1Tenth_Metric_Baked.usd"
    
    from omni.isaac.core.utils.stage import add_reference_to_stage
    add_reference_to_stage(usd_path=usd_path, prim_path="/Full_Car")
    
    # Now initialize view after prim exists
    robots = ArticulationView("/Full_Car")
    world.scene.add(robots)
    
    world.reset()
    robots.initialize()
    
    # Spawn at 50cm
    robots.set_world_poses(positions=np.array([[0, 0, 0.5]]), orientations=np.array([[1, 0, 0, 0]]))
    
    print("[PhysicsTest] Stepping for 100 frames...")
    for i in range(100):
        world.step(render=False)
        if i % 20 == 0:
            pos = robots.get_world_poses()[0][0]
            print(f"Step {i} | Z-Pos: {pos[2]:.4f}m")
            if np.any(np.isnan(pos)):
                print("!!! EXPLOSION DETECTED (NaN) !!!")
                return False
                
    final_z = robots.get_world_poses()[0][0][2]
    print(f"[PhysicsTest] Final Z: {final_z:.4f}m")
    # Expected: ~0.05m (wheel radius)
    if 0.03 < final_z < 0.07:
        print("[PhysicsTest] SUCCESS: Robot settled stably on wheels.")
        return True
    else:
        print(f"[PhysicsTest] FAILURE: Unexpected final height {final_z:.4f}m.")
        return False

if __name__ == "__main__":
    test_physics()
