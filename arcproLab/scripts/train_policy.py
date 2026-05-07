# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Train a Hierarchical SB3 policy for ARCPro Lane Following.")
parser.add_argument("--num_envs", type=int, default=16, help="Number of parallel simulation environments.")
parser.add_argument("--seed", type=int, default=42, help="Seed for the environment.")
parser.add_argument("--total_timesteps", type=int, default=5000000, help="Total timesteps to train (5M recommended for HPPP).")
parser.add_argument("--checkpoint", type=str, default=None, help="Path to a checkpoint to resume from.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import os
import sys
import torch
import warnings
import numpy as np
import gymnasium as gym
import cv2
from datetime import datetime

# Silence gym warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)

# Add both root and arcproLab to sys.path
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
from stable_baselines3.common.callbacks import CheckpointCallback, BaseCallback
from stable_baselines3.common.vec_env import VecNormalize, VecEnv, VecMonitor

from isaaclab.envs import ManagerBasedRLEnv
from arcpro_env_cfg import ARCProEnvCfg

# Submodule Imports
from policies.hierarchical_policy import HierarchicalPathPlanningPolicy
from policies.fusion_policy import FusionFeaturesExtractor
from wrappers.waypoint_tracking_wrapper import WaypointTrackingWrapper

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
        self.prev_action = torch.zeros((self.num_envs, 3), device=self.device)

    def reset(self):
        obs_dict, info = self.venv.reset()
        self.prev_action.zero_()
        return self._process_obs(obs_dict)

    def step_async(self, actions):
        self._async_actions = actions

    def step_wait(self):
        actions_torch = torch.from_numpy(self._async_actions).to(self.device)
        
        # Store prev_action for the reward function (in env.extras)
        self.venv.extras["prev_action"] = self.prev_action.clone()
        
        obs_dict, rewards, terminated, truncated, info = self.venv.step(actions_torch)
        dones = (terminated | truncated).cpu().numpy()
        
        # Update prev_action for next step
        self.prev_action.copy_(actions_torch)
        
        # SB3 expects info to be a list of dicts, one for each env
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
    # 1. Setup Environment Configuration
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.enable_cameras = True 
    env_cfg.__post_init__() 

    # 2. Create Environment
    env = ManagerBasedRLEnv(cfg=env_cfg)
    
    # 3. Wrap for Hierarchical Tracking (Required for auxiliary loss)
    env = WaypointTrackingWrapper(env, env_id=0)
    
    # 4. Bridge to SB3 and HPPP Contract
    env = HPPPDirectBridge(env)
    
    # 4.5. Monitor Episode Statistics
    env = VecMonitor(env)
    
    # 5. Add Normalization
    env = VecNormalize(env, norm_obs=True, norm_reward=True, clip_obs=10.)

    # 6. Define Log Directory
    log_dir = os.path.join("logs", "ppo", datetime.now().strftime("%Y%m%d-%H%M%S"))
    os.makedirs(log_dir, exist_ok=True)

    # 7. Define Policy & Model
    policy_kwargs = dict(
        features_extractor_class=FusionFeaturesExtractor,
        features_extractor_kwargs=dict(features_dim=268),
        lstm_hidden_size=256,
        n_lstm_layers=1,
        enable_critic_lstm=True,
        share_features_extractor=True,
        # Hierarchical parameters
        num_waypoints=5,
        waypoint_horizon=2.5,
        use_kinematic_anchors=True,
    )

    if args_cli.checkpoint:
        print(f"Resuming from checkpoint: {args_cli.checkpoint}")
        checkpoint_dir = os.path.dirname(args_cli.checkpoint)
        norm_file = os.path.join(checkpoint_dir, "vec_normalize.pkl")
        if os.path.exists(norm_file):
            print(f"Loading normalization stats from: {norm_file}")
            env = VecNormalize.load(norm_file, env)
            env.training = True 
            
        model = RecurrentPPO.load(
            args_cli.checkpoint,
            env,
            verbose=1,
            learning_rate=5e-5, # Explicitly override high checkpoint value
            tensorboard_log=os.path.join(log_dir, "tb"),
            seed=args_cli.seed,
            device="cuda"
        )
    else:
        model = RecurrentPPO(
            HierarchicalPathPlanningPolicy,
            env,
            verbose=1,
            learning_rate=2e-5, # Reduced from 5e-5 for stability
            n_steps=2048,      # Increased from 1024 for better gradient estimates
            batch_size=128,    # Increased from 64
            n_epochs=10,
            gamma=0.99,
            gae_lambda=0.95,
            clip_range=0.2,
            ent_coef=0.02,     # Increased from 0.01 to encourage smoother exploration
            tensorboard_log=os.path.join(log_dir, "tb"),
            seed=args_cli.seed,
            device="cuda",
            policy_kwargs=policy_kwargs,
        )

    # 8. Callbacks
    class RewardLoggerCallback(BaseCallback):
        def __init__(self, verbose: int = 0):
            super().__init__(verbose)
        
        def _on_step(self) -> bool:
            if self.n_calls % 1000 == 0:
                info = self.locals.get("infos", [{}])[0]
                ep_rew = info.get("episode", {}).get("r", 0.0)
                ep_len = info.get("episode", {}).get("l", 0)
                print(f"[PROGRESS] Step {self.n_calls} | EpRew: {ep_rew:.2f} | EpLen: {ep_len}")
                sys.stdout.flush()

            return True

    class SaveVecNormalizeCallback(BaseCallback):
        def __init__(self, save_path: str, save_freq: int, verbose: int = 0):
            super().__init__(verbose)
            self.save_path = save_path
            self.save_freq = save_freq

        def _on_step(self) -> bool:
            if self.n_calls % self.save_freq == 0:
                self.training_env.save(os.path.join(self.save_path, "vec_normalize.pkl"))
            return True

    # save_freq is per vectorized step. 1M / num_envs = frequency.
    save_freq = max(1, 1000000 // args_cli.num_envs)
    checkpoint_callback = CheckpointCallback(save_freq=save_freq, save_path=log_dir, name_prefix="model")
    vec_norm_callback = SaveVecNormalizeCallback(save_path=log_dir, save_freq=save_freq)
    reward_logger_callback = RewardLoggerCallback()

    # 9. Train
    print(f"Starting Hierarchical training for {args_cli.total_timesteps} steps...")
    model.learn(
        total_timesteps=args_cli.total_timesteps,
        callback=[checkpoint_callback, vec_norm_callback, reward_logger_callback],
        progress_bar=True
    )

    # 10. Save Final Model
    model.save(os.path.join(log_dir, "model_final"))
    env.save(os.path.join(log_dir, "vec_normalize.pkl"))
    
    # 11. Close
    env.close()

if __name__ == "__main__":
    main()
