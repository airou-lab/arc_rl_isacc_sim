from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

from omni.isaac.core import World
from omni.isaac.core.articulations import ArticulationView
import numpy as np

def validate_core():
    print("[Validate] Initializing Direct Core World...")
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
    
    # Set Gains
    stiffness = np.array([10000.0 if "Knuckle" in n else 0.0 for n in robots.dof_names])
    damping = np.array([1000.0 for _ in robots.dof_names])
    robots.set_gains(stiffness, damping)
    
    # Get Controller
    controller = robots.articulation_controller
    
    # Drive: [13, 16, 24, 25]
    drive_indices = np.array([13, 16, 24, 25], dtype=np.int32)
    vel_targets = np.array([5.0, 5.0, 5.0, 5.0], dtype=np.float32)
    
    print("[Validate] Applying 5 rad/s via ArticulationController.apply_action...")
    
    for i in range(100):
        controller.apply_action(joint_velocities=vel_targets, joint_indices=drive_indices)
        for _ in range(3):
            world.step(render=False)
        
        if i % 20 == 0:
            vel = robots.get_linear_velocities()[0]
            speed = np.linalg.norm(vel[:2])
            z_pos = robots.get_world_poses()[0][0][2]
            print(f"Step {i:3d} | Speed: {speed:.4f} m/s | Z: {z_pos:.4f}m")

    final_vel = np.linalg.norm(robots.get_linear_velocities()[0][:2])
    print(f"[Validate] Final Velocity: {final_vel:.4f} m/s")
    
    if 0.05 < final_vel < 2.0:
        print("[Validate] SUCCESS: CORE PHYSICS ARE STABLE AND RESPONSIVE.")
        return True
    else:
        print(f"[Validate] FAILURE: Robot movement {final_vel:.4f} is unexpected.")
        return False

if __name__ == "__main__":
    validate_core()
