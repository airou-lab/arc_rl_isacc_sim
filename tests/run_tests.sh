#!/bin/bash
# ROS Jazzy on this machine injects an incompatible pytest plugin into kit python;
# autoload must be disabled. See lark/launch_testing ModuleNotFoundError otherwise.
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/arika/IsaacLab/isaaclab.sh -p -m pytest "$(dirname "$0")" -q
