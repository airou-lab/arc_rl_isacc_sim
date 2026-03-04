# setup_ros2_bridge.py
# Run this inside Isaac Sim's Script Editor (Window -> Script Editor)

import carb
import omni
import omni.graph.core as og
from isaacsim.core.utils import extensions

def setup_robot_bridge(robot_path="/mushr_tx2/mushr_fixed/base_footprint"):
    print(f"Setting up ROS2 Bridge for robot at: {robot_path}")
    
    # Enable Extension
    extensions.enable_extension("isaacsim.ros2.bridge")
    
    graph_path = f"{robot_path}/ActionGraph"
    keys = og.Controller.Keys
    
    # Try to delete if it exists
    try:
        import omni.usd
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(graph_path)
        if prim.IsValid():
             print(f"Graph at {graph_path} exists. Overwriting.")
    except:
        pass

    try:
        (graph, nodes, _, _) = og.Controller.edit(
            graph_path,
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("Ros2Context", "isaacsim.ros2.bridge.ROS2Context"),
                    
                    # Subscriber: Receive Commands
                    ("Ros2SubscribeAckermann", "isaacsim.ros2.bridge.ROS2SubscribeAckermannDrive"),
                    
                    # Controller: Move Robot
                    ("AckermannController", "isaacsim.robot.wheeled_robots.AckermannController"),
                    ("SteeringController", "isaacsim.core.nodes.IsaacArticulationController"),
                    ("ThrottleController", "isaacsim.core.nodes.IsaacArticulationController"),
                    
                    # Publisher: Send Vehicle State (Loopback for now)
                    ("Ros2PublishAckermann", "isaacsim.ros2.bridge.ROS2PublishAckermannDrive"),
                    
                    # Camera Publishing
                    ("createViewport", "isaacsim.core.nodes.IsaacCreateViewport"),
                    ("getRenderProduct", "isaacsim.core.nodes.IsaacGetViewportRenderProduct"),
                    ("setCamera", "isaacsim.core.nodes.IsaacSetCameraOnRenderProduct"),
                    ("cameraHelperRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ],
                keys.SET_VALUES: [
                    # Topics
                    ("Ros2SubscribeAckermann.inputs:topicName", "/ackermann_cmd"),
                    ("Ros2PublishAckermann.inputs:topicName", "/vehicle_state"),
                    ("cameraHelperRgb.inputs:topicName", "/camera/image_raw"),
                    ("cameraHelperRgb.inputs:type", "rgb"),
                    
                    # Robot Controllers
                    ("SteeringController.inputs:targetPrim", [robot_path]),
                    ("ThrottleController.inputs:targetPrim", [robot_path]),
                    ("SteeringController.inputs:jointNames", ["front_left_wheel_steer", "front_right_wheel_steer"]),
                    ("ThrottleController.inputs:jointNames", [
                        "front_left_wheel_throttle", "front_right_wheel_throttle",
                        "back_left_wheel_throttle", "back_right_wheel_throttle"
                    ]),
                    
                    # Ackermann Geometry (Mushr)
                    ("AckermannController.inputs:wheelBase", 0.19),
                    ("AckermannController.inputs:trackWidth", 0.18),
                    ("AckermannController.inputs:frontWheelRadius", 0.03),
                    ("AckermannController.inputs:backWheelRadius", 0.03),
                    
                    # Camera
                    ("createViewport.inputs:viewportId", 0),
                ],
                keys.CONNECT: [
                    # Execution Flow
                    ("OnPlaybackTick.outputs:tick", "Ros2SubscribeAckermann.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "Ros2PublishAckermann.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "SteeringController.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "ThrottleController.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "createViewport.inputs:execIn"),
                    
                    # Context
                    ("Ros2Context.outputs:context", "Ros2SubscribeAckermann.inputs:context"),
                    ("Ros2Context.outputs:context", "Ros2PublishAckermann.inputs:context"),
                    ("Ros2Context.outputs:context", "cameraHelperRgb.inputs:context"),

                    # Drive: Command -> Controller
                    ("Ros2SubscribeAckermann.outputs:execOut", "AckermannController.inputs:execIn"),
                    ("Ros2SubscribeAckermann.outputs:speed", "AckermannController.inputs:speed"),
                    ("Ros2SubscribeAckermann.outputs:steeringAngle", "AckermannController.inputs:steeringAngle"),
                    
                    # Drive: Controller -> Joints
                    ("AckermannController.outputs:wheelAngles", "SteeringController.inputs:positionCommand"),
                    ("AckermannController.outputs:wheelRotationVelocity", "ThrottleController.inputs:velocityCommand"),
                    
                    # State Feedback: Command -> Publisher (Loopback for verification)
                    ("Ros2SubscribeAckermann.outputs:speed", "Ros2PublishAckermann.inputs:speed"),
                    ("Ros2SubscribeAckermann.outputs:steeringAngle", "Ros2PublishAckermann.inputs:steeringAngle"),

                    # Camera: Viewport -> Helper
                    ("createViewport.outputs:execOut", "getRenderProduct.inputs:execIn"),
                    ("createViewport.outputs:viewport", "getRenderProduct.inputs:viewport"),
                    ("getRenderProduct.outputs:execOut", "setCamera.inputs:execIn"),
                    ("getRenderProduct.outputs:renderProductPath", "setCamera.inputs:renderProductPath"),
                    ("setCamera.outputs:execOut", "cameraHelperRgb.inputs:execIn"),
                    ("getRenderProduct.outputs:renderProductPath", "cameraHelperRgb.inputs:renderProductPath"),
                ],
            },
        )
        print(f"Successfully created ActionGraph at {graph_path}")
    except Exception as e:
        print(f"Error creating ActionGraph: {e}")

# Run setup
setup_robot_bridge()


# Run setup
setup_robot_bridge()
