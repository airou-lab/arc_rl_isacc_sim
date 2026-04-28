# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
Script to verify the Hierarchical SB3 policy in the Isaac Lab environment.
"""

import argparse
from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Verify the Hierarchical SB3 policy in the Isaac Lab environment.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to spawn.")
parser.add_argument("--max_steps", type=int, default=10000, help="Maximum number of simulation steps.")
parser.add_argument("--checkpoint", type=str, required=True, help="Path to the SB3 checkpoint (.zip).")
parser.add_argument("--debug", action="store_true", help="Enable debug visualizations for markers and track.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()
args_cli.enable_cameras = True 

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch
import os
import sys
import numpy as np
import gymnasium as gym
import cv2

# Add both root, arcproLab, and policy_stack to sys.path
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))
ARCPRO_LAB_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
POLICY_STACK_DIR = os.path.join(ARCPRO_LAB_DIR, "policy_stack")

if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)
if ARCPRO_LAB_DIR not in sys.path:
    sys.path.insert(0, ARCPRO_LAB_DIR)
if POLICY_STACK_DIR not in sys.path:
    sys.path.insert(0, POLICY_STACK_DIR)

from sb3_contrib import RecurrentPPO
from stable_baselines3.common.vec_env import VecNormalize, VecEnv

from isaaclab.envs import ManagerBasedRLEnv
from arcpro_env_cfg import ARCProEnvCfg

class HPPPDirectBridge(VecEnv):
    """
    Bridge between Isaac Lab and HPPP Brain.
    Acts as a Stable Baselines3 VecEnv and preserves the Dict observation space.
    """
    def __init__(self, env: gym.Env):
        self.venv = env
        self.num_envs = env.get_wrapper_attr("num_envs")
        self.device = env.get_wrapper_attr("device")
        
        # Note: In modern Isaac Lab, groups with single terms are already Boxes
        single_obs_space = env.get_wrapper_attr("single_observation_space")
        img_space = single_obs_space.spaces["visual"]
        
        transposed_img_space = gym.spaces.Box(
            low=0.0, high=1.0, 
            shape=(img_space.shape[2], img_space.shape[0], img_space.shape[1]),
            dtype=np.float32
        )
        observation_space = gym.spaces.Dict({
            "vec": single_obs_space.spaces["policy"],
            "image": transposed_img_space
        })
        
        # Override Isaac Lab's unbounded ActionManager space with strict SB3 bounds
        low = np.array([-1.0, 0.0, 0.0], dtype=np.float32)
        high = np.array([1.0, 1.0, 1.0], dtype=np.float32)
        action_space = gym.spaces.Box(low=low, high=high, dtype=np.float32)
        
        super().__init__(self.num_envs, observation_space, action_space)
        
        self.render_mode = None
        self.metadata = {"render_modes": []}

    def reset(self):
        obs_dict, info = self.venv.reset()
        return self._process_obs(obs_dict)

    def step_async(self, actions):
        self._async_actions = actions

    def step_wait(self):
        actions_torch = torch.from_numpy(self._async_actions).to(self.device)
        obs_dict, rewards, terminated, truncated, info = self.venv.step(actions_torch)
        dones = (terminated | truncated).cpu().numpy()
        
        infos = [{} for _ in range(self.num_envs)]
        for key, value in info.items():
            if isinstance(value, torch.Tensor) and value.shape[0] == self.num_envs:
                val_np = value.cpu().numpy()
                for i in range(self.num_envs):
                    infos[i][key] = val_np[i]
            else:
                for i in range(self.num_envs):
                    infos[i][key] = value
                    
        return self._process_obs(obs_dict), rewards.cpu().numpy(), dones, infos

    def _process_obs(self, obs_dict):
        img = obs_dict["visual"].cpu().numpy()
        # Transpose from (B, H, W, C) to (B, C, H, W)
        img = np.transpose(img, (0, 3, 1, 2))
        return {
            "vec": obs_dict["policy"].cpu().numpy(),
            "image": img
        }

    def close(self):
        self.venv.close()

    def get_attr(self, attr_name, indices=None):
        val = getattr(self.venv, attr_name, None)
        return [val] * self.num_envs

    def set_attr(self, attr_name, value, indices=None):
        setattr(self.venv, attr_name, value)

    def env_method(self, method_name, *method_args, indices=None, **method_kwargs):
        method = getattr(self.venv, method_name)
        val = method(*method_args, **method_kwargs)
        return [val] * self.num_envs

    def env_is_wrapped(self, wrapper_class, indices=None):
        return [False] * self.num_envs

    @property
    def base_env(self):
        return self.venv

def main():
    # 1. Setup configuration
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.enable_cameras = True 
    env_cfg.__post_init__() 
    
    # 2. Setup environment
    print("Initializing ManagerBasedRLEnv...")
    env = ManagerBasedRLEnv(cfg=env_cfg)
    
    # 3. Bridge to HPPP Contract
    env = HPPPDirectBridge(env)
    
    # 4. Load Normalization stats
    checkpoint_dir = os.path.dirname(args_cli.checkpoint)
    norm_file = os.path.join(checkpoint_dir, "vec_normalize.pkl")
    if os.path.exists(norm_file):
        print(f"Loading normalization stats from: {norm_file}")
        env = VecNormalize.load(norm_file, env)
        env.training = False
        env.norm_reward = False
    else:
        print(f"Warning: No normalization stats found at {norm_file}. Using defaults.")
        env = VecNormalize(env, norm_obs=True, norm_reward=False, clip_obs=10.)
    
    # 5. Load model
    print(f"Loading Hierarchical SB3 policy from: {args_cli.checkpoint}")
    model = RecurrentPPO.load(args_cli.checkpoint, env=env, device="cuda" if torch.cuda.is_available() else "cpu")
    print("Policy loaded.")
    
    # 6. Reset environment
    print("Resetting environment...")
    obs = env.reset()
    
    # LSTM states initialization
    lstm_states = None
    episode_starts = np.ones((args_cli.num_envs,), dtype=bool)

    # 7. Simulation loop
    count = 0
    max_steps = args_cli.max_steps
    print(f"Starting simulation loop for {max_steps} steps...")
    
    try:
        while simulation_app.is_running() and count < max_steps:
            # Predict action with LSTM states
            actions, lstm_states = model.predict(
                obs, 
                state=lstm_states, 
                episode_start=episode_starts, 
                deterministic=True
            )
            
            # Step environment
            obs, rewards, dones, infos = env.step(actions)
            
            # Action Debug
            # actions is (B, 3) -> [steer, throttle, brake]
            steer = actions[0, 0]
            throttle = actions[0, 1]
            brake = actions[0, 2]
            if count % 5 == 0:
                print(f"  [ACT] Step {count:4d} | Steer: {steer:6.3f} | Throttle: {throttle:6.3f} | Brake: {brake:6.3f}")

            # Save debug frames for the first 5 steps
            if count < 5:
                debug_dir = "debug_frames"
                os.makedirs(debug_dir, exist_ok=True)
                # obs is from VecNormalize -> HPPPDirectBridge
                # obs["image"] is (B, C, H, W). We want the first env [0]
                img_np = obs["image"][0] # (C, H, W)
                # Convert from CHW to HWC for cv2
                img_hwc = np.transpose(img_np, (1, 2, 0))
                # It's normalized [0, 1] float, convert to [0, 255] uint8
                img_uint8 = (img_hwc * 255).astype(np.uint8)
                # Convert RGB to BGR for cv2
                img_bgr = cv2.cvtColor(img_uint8, cv2.COLOR_RGB2BGR)
                cv2.imwrite(os.path.join(debug_dir, f"frame_{count}.png"), img_bgr)
                print(f"  [DEBUG] Saved camera frame {count} to {debug_dir}/frame_{count}.png")

            # Update episode starts for LSTM reset logic
            episode_starts = dones
            
            count += 1
            if count % 100 == 0:
                print(f"Step {count}/{max_steps}")
                
            # Quick telemetry check
            if count % 20 == 0:
                # The env chain is VecNormalize -> HPPPDirectBridge -> WaypointTrackingWrapper -> ManagerBasedRLEnv
                raw_env = env.venv.venv.unwrapped
                raw_lat_err = raw_env.extras.get("lat_err", torch.tensor([0.0]))[0].item()
                print(f"  [Tele] LatErr: {raw_lat_err:6.3f}m")
                
            if dones[0]:
                print(f"Episode terminated at step {count}.")
                
            simulation_app.update()
                
    except KeyboardInterrupt:
        print("Simulation interrupted by user.")
    except Exception as e:
        print(f"Exception caught in simulation loop: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("Closing environment...")
        env.close()
        simulation_app.close()

if __name__ == "__main__":
    main()
