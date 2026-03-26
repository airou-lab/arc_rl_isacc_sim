# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import torch
import torchvision.models as models
import torch.nn as nn
from torchvision import transforms
import os
from typing import Tuple

class PolicyWrapper:
    """
    A wrapper for the ResNet18 road-following model (ackermann_amr_policy).
    
    This class loads the pre-trained ResNet18-based model and prepares it for inference.
    The model takes a 224x224 RGB image and outputs a (x, y) coordinate in range [-1, 1].
    """

    def __init__(self, model_path: str):
        """
        Initializes the PolicyWrapper and loads the model.

        Args:
            model_path (str): The path to the saved .pth model file.
        """
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"Model file not found at: {model_path}")

        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        
        # Replicate the model structure from ackermann_amr_policy.py
        full_model = models.resnet18(weights=None)
        self.model = nn.Sequential(*list(full_model.children())[:-1])
        self.fc = nn.Linear(512, 2)
        self.sigma = nn.Tanh()
        
        # Load the trained weights
        state_dict = torch.load(model_path, map_location=self.device)
        
        # The state_dict might have keys like 'model.0.weight' or 'fc.weight'
        # We need to map them correctly to our local self.model and self.fc
        # Based on inspection, keys are 'model.0.weight', ..., 'fc.weight'
        
        # We can wrap our parts in a temporary module to load the state_dict directly if keys match
        class TemporaryModule(nn.Module):
            def __init__(self, model_part, fc_part):
                super().__init__()
                self.model = model_part
                self.fc = fc_part
        
        temp_module = TemporaryModule(self.model, self.fc)
        temp_module.load_state_dict(state_dict)
        
        temp_module.to(self.device)
        temp_module.eval()
        self.inference_model = temp_module

        # Image Processing Transform (excluding ColorJitter for inference)
        self.transform = transforms.Compose([
            transforms.Resize((224, 224)),
            transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
        ])

    def predict(self, image: torch.Tensor) -> Tuple[float, float]:
        """
        Performs inference on a single image.

        Args:
            image (torch.Tensor): The raw image tensor from the simulator (H, W, 3) or (1, 3, H, W).
                                 Expected range [0, 255] or [0, 1].

        Returns:
            Tuple[float, float]: The (x, y) coordinate predicted by the model in range [-1, 1].
        """
        with torch.no_grad():
            # 1. Ensure correct format (Batch, Channels, Height, Width)
            if image.ndim == 3:
                # Assuming (H, W, C) -> (1, C, H, W)
                image = image.permute(2, 0, 1).unsqueeze(0)
            elif image.ndim == 4:
                # Assuming (B, H, W, C) -> (B, C, H, W)
                if image.shape[1] != 3:
                    image = image.permute(0, 3, 1, 2)
            
            # 2. Convert to float and scale to [0, 1] if needed
            if image.dtype == torch.uint8:
                image = image.float() / 255.0
            
            # 3. Apply transforms
            image = self.transform(image).to(self.device)
            
            # 4. Forward pass
            x = self.inference_model.model(image)
            x = x.view(x.size(0), -1)
            output = self.sigma(self.inference_model.fc(x))
            
            # 5. Convert to tuple of floats
            x_out, y_out = output.squeeze().cpu().numpy()
            return float(x_out), float(y_out)

    def get_action(self, prediction: Tuple[float, float]) -> Tuple[float, float]:
        """
        Converts model prediction (x, y) into steering and throttle.

        Args:
            prediction (Tuple[float, float]): The (x, y) output from the model.

        Returns:
            Tuple[float, float]: (steering_angle, throttle_rad_s)
        """
        prediction_x, prediction_y = prediction
        
        # Steering Logic from model_evaluate.py
        forward = torch.tensor([0.0, -1.0])
        offset = torch.tensor([0.0, 1.0])
        traj = torch.tensor([prediction_x, prediction_y]) - offset
        
        # Avoid division by zero
        norm = torch.linalg.norm(traj)
        if norm < 1e-6:
            unit_traj = torch.tensor([0.0, -1.0])
        else:
            unit_traj = traj / norm
            
        unit_forward = forward # already unit
        
        # Dot product
        dot_prod = torch.dot(unit_traj, unit_forward)
        dot_prod = torch.clamp(dot_prod, -1.0, 1.0)
        
        # steering_angle = arccos(dot) / 2.0 - deg2rad(15.0)
        # Note: arccos returns positive value in [0, pi].
        # We need to know the sign (left/right).
        angle = torch.acos(dot_prod) / 2.0
        if prediction_x > 0:
            angle = -angle 
        
        # Bias from original code
        steering_angle = angle - torch.deg2rad(torch.tensor(15.0))
        
        # Throttle logic: constant speed as in model_evaluate.py
        # speed = 2.0 m/s
        # wheel radius is 0.05m (from arcpro_robot_cfg.py)
        # rad/s = speed / radius = 2.0 / 0.05 = 40.0
        throttle_rad_s = 40.0
        
        return float(steering_angle), float(throttle_rad_s)

# Example usage (for testing)
if __name__ == "__main__":
    import sys
    # Add the current directory to sys.path to allow imports from the venv
    sys.path.append("./.venv/lib/python3.12/site-packages")

    relative_model_path = "arcproLab/models/road_following_model.pth"
    
    try:
        policy = PolicyWrapper(relative_model_path)
        print("PolicyWrapper initialized successfully.")
        
        # Create a dummy image tensor (160, 90, 3) - simulating raw simulator output
        dummy_image = torch.randint(0, 256, (90, 160, 3), dtype=torch.uint8)
        
        # Perform prediction
        prediction = policy.predict(dummy_image)
        print(f"Prediction on dummy data: {prediction}")

    except FileNotFoundError as e:
        print(f"Error: {e}")
    except Exception as e:
        import traceback
        traceback.print_exc()
