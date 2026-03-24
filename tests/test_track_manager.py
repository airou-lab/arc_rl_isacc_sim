import pytest
import torch
import numpy as np
import os
import sys
from unittest.mock import MagicMock

# Add arcproLab to sys.path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "arcproLab"))

from mdp.track_manager import TrackManager

@pytest.fixture
def mock_track_manager():
    # Mock omni and pxr before creating TrackManager
    sys.modules["omni"] = MagicMock()
    sys.modules["omni.usd"] = MagicMock()
    sys.modules["pxr"] = MagicMock()
    
    tm = TrackManager(device="cpu")
    # Provide synthetic waypoints for testing
    # Straight line along X axis
    wps = np.zeros((10, 3))
    wps[:, 0] = np.linspace(0, 9, 10)
    wps[:, 2] = 0.0 # Facing +X
    tm.waypoints = torch.tensor(wps, device="cpu", dtype=torch.float32)
    return tm

def test_closest_waypoint(mock_track_manager):
    # Robot at (2.1, 0.5)
    pos = torch.tensor([[2.1, 0.5, 0.0]], device="cpu")
    closest = mock_track_manager.get_closest_waypoint_data(pos)
    # Should be waypoint at index 2: (2.0, 0.0, 0.0)
    assert closest[0, 0] == 2.0
    assert closest[0, 1] == 0.0

def test_compute_errors_straight(mock_track_manager):
    # Robot at (5.0, 0.5) facing +X
    pos = torch.tensor([[5.0, 0.5, 0.0]], device="cpu")
    yaw = torch.tensor([0.0], device="cpu")
    
    lat_err, head_err = mock_track_manager.compute_errors(pos, yaw)
    
    # Lateral error: robot is at Y=0.5, track is at Y=0.0. 
    # Normal vector for yaw=0 is (0, 1). 
    # dot((0, 0.5), (0, 1)) = 0.5
    assert torch.allclose(lat_err, torch.tensor([0.5]))
    assert torch.allclose(head_err, torch.tensor([0.0]))

def test_compute_errors_angled(mock_track_manager):
    # Robot at (5.0, 0.0) facing 0.1 rad off track
    pos = torch.tensor([[5.0, 0.0, 0.0]], device="cpu")
    yaw = torch.tensor([0.1], device="cpu")
    
    lat_err, head_err = mock_track_manager.compute_errors(pos, yaw)
    
    assert torch.allclose(lat_err, torch.tensor([0.0]), atol=1e-6)
    assert torch.allclose(head_err, torch.tensor([0.1]))
