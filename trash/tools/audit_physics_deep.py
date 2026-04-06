import sys
import os
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(CURRENT_DIR)
sys.path.append(os.path.join(CURRENT_DIR, "arcproLab"))

import argparse
from isaaclab.app import AppLauncher

# Generate a minimal Isaac Lab app to access PhysX and USD
parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
# Set headless to True to avoid GUI issues in CLI
args_cli.headless = True
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import os
import torch
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema
from isaaclab.sim import SimulationCfg
import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg
from isaaclab.scene import InteractiveSceneCfg, InteractiveScene
from arcpro_robot_cfg import ARCPRO_ROBOT_CFG

def deep_audit():
    print("\n" + "="*50)
    print("DEEP PHYSICS AUDIT: F1TENTH IMMOBILITY DIAGNOSIS")
    print("="*50)

    # 1. Check USD Asset directly
    usd_path = os.path.join(os.path.dirname(__file__), "arcproLab", "assets", "robot", "F1Tenth_Metric.usd")
    print(f"\n[1] Auditing USD Asset: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    if not stage:
        print("FAILED TO OPEN USD")
    else:
        for prim in stage.Traverse():
            if prim.IsA(UsdGeom.Xformable):
                props = prim.GetPropertyNames()
                if "physics:mass" in props:
                    val = prim.GetAttribute("physics:mass").Get()
                    print(f"  - Prim {prim.GetPath()} has mass: {val}")
                
                # Check for Collision API
                if prim.HasAPI(UsdPhysics.CollisionAPI):
                    print(f"  - Prim {prim.GetPath()} HAS CollisionAPI")
                    enabled = prim.GetAttribute("physics:collisionEnabled").Get()
                    print(f"    Enabled: {enabled}")

    # 2. Check Runtime Configuration
    print(f"\n[2] Auditing Runtime Config (arcpro_robot_cfg.py)")
    print(f"  - Configured Mass: {ARCPRO_ROBOT_CFG.spawn.mass_props.mass}")
    print(f"  - Solver Iterations (Pos/Vel): {ARCPRO_ROBOT_CFG.spawn.articulation_props.solver_position_iteration_count}/{ARCPRO_ROBOT_CFG.spawn.articulation_props.solver_velocity_iteration_count}")
    
    for act_name, act_cfg in ARCPRO_ROBOT_CFG.actuators.items():
        print(f"  - Actuator '{act_name}':")
        print(f"    Stiffness: {act_cfg.stiffness}")
        print(f"    Damping: {act_cfg.damping}")
        print(f"    Effort Limit: {act_cfg.effort_limit_sim}")

    # 3. Simulate and Check Contacts (Simplified)
    print(f"\n[3] Instantiating Scene for Contact Check...")
    scene_cfg = InteractiveSceneCfg(num_envs=1, env_spacing=1.0)
    # Manual robot spec to match arcpro_env_cfg
    scene_cfg.robot = ARCPRO_ROBOT_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot",
        spawn=ARCPRO_ROBOT_CFG.spawn.replace(
            usd_path=usd_path,
            scale=(8.0, 8.0, 8.0), # Matches current arcpro_env_cfg.py
        )
    )
    # Add ground
    from isaaclab.assets import AssetBaseCfg
    scene_cfg.ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(),
    )
    
    # Use real config scale (8.0x) and no overrides
    scene_cfg.robot.spawn.scale = (8.0, 8.0, 8.0) 

    sim_cfg = SimulationCfg(dt=0.01, device="cuda:0")
    sim = sim_utils.SimulationContext(sim_cfg)
    
    scene = InteractiveScene(scene_cfg)
    sim.reset()
    print("  - Simulation Context Initialized.")
    
    robot = scene["robot"]
    print(f"  - Robot spawned at scale: {robot.data.root_state_w[:, 7:10]}") # Just checking if scale is reflected in data if possible, though scale isn't in root_state usually
    
    # Check joint positions
    print(f"  - Joint Positions: {robot.data.joint_pos}")
    
    # Step simulation once to let gravity/collisions settle
    for _ in range(10):
        sim.step()
    
    print(f"\n[4] Post-Step State:")
    print(f"  - Root Position (Z): {robot.data.root_pos_w[0, 2]:.4f}")
    
    # If Z is very low, it might be clipping
    if robot.data.root_pos_w[0, 2] < 0.05:
        print("  - [!] WARNING: Robot Z-height is very low. Possible clipping into ground.")
    
    # Apply full throttle
    print(f"\n[5] Testing Drive Effort...")
    actions = torch.zeros(scene.num_envs, robot.num_joints, device=sim.device)
    # Find throttle joints
    throttle_indices, _ = robot.find_joints("Joint_Drive_.*")
    actions[:, throttle_indices] = 100.0 # High velocity target
    
    robot.set_joint_velocity_target(actions[:, throttle_indices], joint_ids=throttle_indices)
    
    for i in range(50):
        sim.step()
        if i % 10 == 0:
            vel = torch.mean(torch.norm(robot.data.root_lin_vel_w, dim=-1)).item()
            print(f"  - Step {i}: Linear Velocity = {vel:.4f}")

    final_vel = torch.mean(torch.norm(robot.data.root_lin_vel_w, dim=-1)).item()
    if final_vel < 0.01:
        print("\n[!] DIAGNOSIS: ROBOT IS IMMOBILE despite 100.0 velocity target.")
    else:
        print(f"\n[+] SUCCESS: Robot moved with velocity {final_vel:.4f}")

if __name__ == "__main__":
    deep_audit()
    simulation_app.close()
