from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

from omni.isaac.core import World
from omni.isaac.core.articulations import ArticulationView
import numpy as np

def validate_hardened():
    print("[Validate] Initializing Hardened Metric World...")
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
    kps = np.array([10000.0 if "Knuckle" in n else 0.0 for n in robots.dof_names])
    kds = np.array([5000.0 for _ in robots.dof_names])
    robots.set_gains(kps=kps, kds=kds)
    
    # Spawn at 10cm
    robots.set_world_poses(positions=np.array([[0, 0, 0.1]]), orientations=np.array([[1, 0, 0, 0]]))
    
    drive_indices = [13, 16, 24, 25]
    targets = np.array([[10.0, 10.0, 10.0, 10.0]], dtype=np.float32)
    
    print("[Validate] Applying persistent 10 rad/s to drive wheels...")
    
    for i in range(100):
        for _ in range(3):
            robots.set_joint_velocity_targets(targets, joint_indices=drive_indices)
            world.step(render=False)
        
        if i % 20 == 0:
            vel = robots.get_linear_velocities()[0]
            speed = np.linalg.norm(vel[:2])
            z_pos = robots.get_world_poses()[0][0][2]
            print(f"Step {i:3d} | Speed: {speed:.4f} m/s | Z: {z_pos:.4f}m")

    final_vel = np.linalg.norm(robots.get_linear_velocities()[0][:2])
    print(f"[Validate] Final Velocity: {final_vel:.4f} m/s")
    
    simulation_app.close()
    
    if final_vel > 0.1:
        print("[Validate] SUCCESS: ROBOT IS STABLE AND MOVING.")
        return True
    else:
        print("[Validate] FAILURE: Movement not detected.")
        return False

if __name__ == "__main__":
    validate_hardened()
