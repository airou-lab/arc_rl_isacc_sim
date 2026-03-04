
import os
from isaacsim import SimulationApp

# Initialize simulation app
simulation_app = SimulationApp({"headless": True})

from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
import omni.usd
from pxr import UsdPhysics

# Load stage
usd_path = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/isacc_sim_usd/World0.usd"
print(f"Loading stage: {usd_path}")
omni.usd.get_context().open_stage(usd_path)
simulation_app.update()

# Initialize World
world = World()
world.reset()

robot_path = "/World/F1Tenth"
robot = Articulation(robot_path)
world.scene.add(robot)
world.reset()

print(f"\n--- Robot Properties: {robot_path} ---")
# 1. Check Mass using pxr directly since utils is failing
stage = omni.usd.get_context().get_stage()
total_mass = 0
for prim in stage.Traverse():
    if prim.HasAPI(UsdPhysics.MassAPI):
        mass_api = UsdPhysics.MassAPI(prim)
        m = mass_api.GetMassAttr().Get()
        if m: total_mass += m
print(f"Estimated Total Mass from USD: {total_mass} kg")

# 2. Check Drive Targets and gains
print("\n--- Current Joint Drive States ---")
for i in range(robot.num_dof):
    name = robot.dof_names[i]
    print(f"DOF {i} ({name}):")
    prim = stage.GetPrimAtPath(f"{robot_path}/Joints/{name}")
    drive = UsdPhysics.DriveAPI.Get(prim, "angular")
    if drive:
        print(f"  Drive Type: {drive.GetTypeAttr().Get()}")
        print(f"  Max Force: {drive.GetMaxForceAttr().Get()}")
        print(f"  Stiffness (Kp): {drive.GetStiffnessAttr().Get()}")
        print(f"  Damping (Kd): {drive.GetDampingAttr().Get()}")
        print(f"  Target Velocity: {drive.GetTargetVelocityAttr().Get()}")
        print(f"  Target Position: {drive.GetTargetPositionAttr().Get()}")

simulation_app.close()
