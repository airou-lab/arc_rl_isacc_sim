from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

from omni.isaac.core import World
from omni.isaac.core.articulations import ArticulationView
import numpy as np

def validate_metric_control():
    print("[Validate] Initializing Metric World...")
    world = World(stage_units_in_meters=1.0)
    world.get_physics_context().set_gravity(-9.81)
    world.scene.add_default_ground_plane()
    
    usd_path = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/f1tenth_trainer/assets/F1Tenth_Metric_Baked.usd"
    from omni.isaac.core.utils.stage import add_reference_to_stage
    add_reference_to_stage(usd_path=usd_path, prim_path="/Full_Car")
    
    robots = ArticulationView("/Full_Car")
    world.scene.add(robots)
    world.reset()
    robots.initialize()
    
    # Drive: [13, 16, 24, 25] 
    drive_indices = np.array([13, 16, 24, 25])
    
    print("[Validate] Applying 10 rad/s to wheels for 100 steps...")
    
    for i in range(100):
        # Direct Velocity Set for Validation
        # (Using view indices [0] for the first/only robot)
        vel_targets = np.zeros((1, robots.num_dof))
        vel_targets[0, drive_indices] = 20.0 # Higher speed for obvious movement
        
        robots.set_joint_velocities(vel_targets)
        world.step(render=False)
        
        if i % 20 == 0:
            vel = robots.get_linear_velocities()[0]
            speed = np.linalg.norm(vel[:2])
            z_pos = robots.get_world_poses()[0][0][2]
            print(f"Step {i:3d} | Speed: {speed:.4f} m/s | Z: {z_pos:.4f}m")

    final_vel = np.linalg.norm(robots.get_linear_velocities()[0][:2])
    print(f"[Validate] Final Velocity: {final_vel:.4f} m/s")
    
    if final_vel > 0.1:
        print("[Validate] SUCCESS: Metric robot is responsive to control.")
        return True
    else:
        print("[Validate] FAILURE: Robot did not move.")
        return False

if __name__ == "__main__":
    validate_metric_control()
