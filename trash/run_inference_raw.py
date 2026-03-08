
import os
import sys
import numpy as np
import torch
from sb3_contrib import RecurrentPPO

# Add policy directory
POLICY_DIR = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_policy"
if POLICY_DIR not in sys.path:
    sys.path.append(POLICY_DIR)

from isaac_direct_env import IsaacDirectEnv

MODEL_PATH = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_policy/models/isaac_hppo_20260223_210131/final_model.zip"

def run():
    print(f"[Raw Inference] Loading: {MODEL_PATH}")
    env = IsaacDirectEnv(headless=False)
    
    # Load model
    model = RecurrentPPO.load(MODEL_PATH)
    policy = model.policy
    policy.eval()
    
    obs, info = env.reset()
    
    # LSTM states: tuple of (hidden, cell) for policy and value function
    # RecurrentPPO policy has lstm_states for 'pi' and 'vf'
    # We only need 'pi' for inference
    lstm_states = None
    
    print("[Raw Inference] Starting loop...")
    for i in range(100):
        # 1. Prepare Image: (H, W, 3) uint8 -> (1, 3, H, W) float32 [0, 1]
        img = obs["image"]
        img_tensor = torch.from_numpy(img).permute(2, 0, 1).float() / 255.0
        img_tensor = img_tensor.unsqueeze(0).to(policy.device)
        
        # 2. Prepare Vector: (12,) -> (1, 12) float32
        vec = obs["vec"]
        vec_tensor = torch.from_numpy(vec).float().unsqueeze(0).to(policy.device)
        
        # 3. Combine into observation dict
        obs_dict = {"image": img_tensor, "vec": vec_tensor}
        
        # 4. Predict
        # episode_start mask
        episode_start = torch.tensor([i == 0], dtype=torch.float32).to(policy.device)
        
        with torch.no_grad():
            # RecurrentPPO.policy.forward() takes obs, state, episode_start
            # But we want the action. Better use policy.predict_values or policy.get_distribution
            # Or just use the model's predict but pass tensors? No, model.predict takes numpy.
            
            # Let's use the internal _predict
            actions, lstm_states = policy.predict(obs_dict, state=lstm_states, episode_start=episode_start, deterministic=True)
        
        action = actions.cpu().numpy().flatten()
        
        if i % 10 == 0:
            print(f"Step {i} | Action: {action} | Img Mean: {np.mean(img):.1f}")
            
        obs, reward, terminated, truncated, info = env.step(action)
        if terminated or truncated:
            break

    env.close()

if __name__ == "__main__":
    run()
