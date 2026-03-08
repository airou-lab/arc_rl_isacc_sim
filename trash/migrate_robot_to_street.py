
import os
import numpy as np
from isaacsim import SimulationApp

# 1. Start Simulation App
simulation_app = SimulationApp({"headless": True})

import omni.usd
from isaacsim.core.utils import extensions
import omni.graph.core as og
from pxr import Usd, UsdGeom, Gf

# 2. Enable ROS2 Bridge extension
extensions.enable_extension("isaacsim.ros2.bridge")

def migrate():
    new_world_path = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/openStreetUSD/arcpro_RL_open_street_sim.usd"
    robot_source = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/f1tenth_trainer/assets/F1Tenth.usd"
    
    print(f"Loading Target World: {new_world_path}")
    omni.usd.get_context().open_stage(new_world_path)
    simulation_app.update()
    
    stage = omni.usd.get_context().get_stage()
    
    # 3. Add Robot Reference
    robot_path = "/World/F1Tenth"
    from omni.isaac.core.utils.stage import add_reference_to_stage
    if not stage.GetPrimAtPath(robot_path).IsValid():
        print(f"Adding Robot Reference to {robot_path}...")
        add_reference_to_stage(usd_path=robot_source, prim_path=robot_path)
    
    # Position robot
    robot_prim = stage.GetPrimAtPath(robot_path)
    xform = UsdGeom.Xformable(robot_prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.1))

    # 4. Setup Action Graph
    print("Setting up Action Graph for ROS2 Bridge...")
    graph_path = f"{robot_path}/ActionGraph"
    
    if stage.GetPrimAtPath(graph_path).IsValid():
        stage.RemovePrim(graph_path)

    keys = og.Controller.Keys
    (graph, nodes, _, _) = og.Controller.edit(
        graph_path,
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("Ros2Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("Ros2SubscribeAckermann", "isaacsim.ros2.bridge.ROS2SubscribeAckermannDrive"),
                ("AckermannController", "isaacsim.robot.wheeled_robots.AckermannController"),
                ("SteeringController", "isaacsim.core.nodes.IsaacArticulationController"),
                ("ThrottleController", "isaacsim.core.nodes.IsaacArticulationController"),
                ("createViewport", "isaacsim.core.nodes.IsaacCreateViewport"),
                ("getRenderProduct", "isaacsim.core.nodes.IsaacGetViewportRenderProduct"),
                ("setCamera", "isaacsim.core.nodes.IsaacSetCameraOnRenderProduct"),
                ("cameraHelperRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ],
            keys.SET_VALUES: [
                ("Ros2SubscribeAckermann.inputs:topicName", "/ackermann_cmd"),
                ("SteeringController.inputs:robotPath", robot_path),
                ("ThrottleController.inputs:robotPath", robot_path),
                ("SteeringController.inputs:jointNames", ["Knuckle__Upright__Front_Left", "Knuckle__Upright__Front_Right"]),
                ("ThrottleController.inputs:jointNames", ["Wheel__Knuckle__Front_Left", "Wheel__Knuckle__Front_Right", "Wheel__Upright__Rear_Left", "Wheel__Upright__Rear_Right"]),
                ("AckermannController.inputs:wheelBase", 0.33), 
                ("AckermannController.inputs:trackWidth", 0.28),
                ("AckermannController.inputs:frontWheelRadius", 0.05),
                ("AckermannController.inputs:backWheelRadius", 0.05),
                # FIX: Set the camera prim path using target format [path]
                ("setCamera.inputs:cameraPrim", [f"{robot_path}/Rigid_Bodies/Chassis/Camera_Left"]),
                ("cameraHelperRgb.inputs:topicName", "/camera/image_raw"),
                ("cameraHelperRgb.inputs:type", "rgb"),
                ("createViewport.inputs:viewportId", 0),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "Ros2SubscribeAckermann.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "SteeringController.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ThrottleController.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "createViewport.inputs:execIn"),
                ("Ros2Context.outputs:context", "Ros2SubscribeAckermann.inputs:context"),
                ("Ros2SubscribeAckermann.outputs:execOut", "AckermannController.inputs:execIn"),
                ("Ros2SubscribeAckermann.outputs:speed", "AckermannController.inputs:speed"),
                ("Ros2SubscribeAckermann.outputs:steeringAngle", "AckermannController.inputs:steeringAngle"),
                ("AckermannController.outputs:wheelAngles", "SteeringController.inputs:positionCommand"),
                ("AckermannController.outputs:wheelRotationVelocity", "ThrottleController.inputs:velocityCommand"),
                ("createViewport.outputs:execOut", "getRenderProduct.inputs:execIn"),
                ("createViewport.outputs:viewport", "getRenderProduct.inputs:viewport"),
                ("getRenderProduct.outputs:execOut", "setCamera.inputs:execIn"),
                ("getRenderProduct.outputs:renderProductPath", "setCamera.inputs:renderProductPath"),
                ("setCamera.outputs:execOut", "cameraHelperRgb.inputs:execIn"),
                ("getRenderProduct.outputs:renderProductPath", "cameraHelperRgb.inputs:renderProductPath"),
            ],
        },
    )

    # 5. Save the World
    print("Saving Updated New World...")
    omni.usd.get_context().save_stage()
    print("\nSUCCESS: Robot and ActionGraph migrated to the new street world.")

migrate()
simulation_app.close()
