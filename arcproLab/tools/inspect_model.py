import torch
import sys

# Add the current directory to sys.path to allow imports from the venv
sys.path.append("./.venv/lib/python3.12/site-packages")

model_path = "f1tenth_trainer/ros2_f1_tenth_trainer/road_following_model.pth"

try:
    model = torch.load(model_path, map_location=torch.device('cpu'))
    print(model)
except Exception as e:
    print(f"An error occurred: {e}")
