import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Train SKRL AAC policy for ARCPro Lane Following.")
parser.add_argument("--num_envs", type=int, default=16, help="Number of parallel simulation environments.")
parser.add_argument("--seed", type=int, default=42, help="Seed for the environment.")
parser.add_argument("--total_timesteps", type=int, default=5000000, help="Total timesteps to train.")
parser.add_argument("--resume", type=str, default="", help="Path to checkpoint to resume from.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import os
import sys
import torch
import warnings
from datetime import datetime

warnings.filterwarnings("ignore", category=DeprecationWarning)

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))
ARCPRO_LAB_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)
if ARCPRO_LAB_DIR not in sys.path:
    sys.path.insert(0, ARCPRO_LAB_DIR)

from isaaclab.envs import ManagerBasedRLEnv
from arcpro_env_cfg import ARCProEnvCfg

from skrl.envs.wrappers.torch import IsaacLabWrapper, Wrapper
from skrl.agents.torch.ppo import PPO, PPO_DEFAULT_CONFIG
from skrl.trainers.torch import SequentialTrainer
from skrl.memories.torch import RandomMemory
import numpy as np
import gymnasium as gym

from torchvision.models import resnet18, ResNet18_Weights

from agents.skrl_wrappers import SKRLFlattenWrapper

# Import the new SKRL models
from agents.skrl_models import ARCProActor, ARCProCritic

class TelemetryPPO(PPO):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._print_counter = 0
        self._last_ep_len = 0.0
        self._last_ep_rew = 0.0

    def record_transition(self, states, actions, rewards, next_states, terminated, truncated, infos, timestep, timesteps):
        super().record_transition(states, actions, rewards, next_states, terminated, truncated, infos, timestep, timesteps)
        
        speed = 0.0
        track_pct = 0.0
        wp_delta = 0
        if "speed_mps" in infos:
            speed = infos["speed_mps"].mean().item()
            self.track_data("Telemetry / Speed_MPS", speed)
        if "dist_white" in infos:
            self.track_data("Telemetry / Dist_White_m", infos["dist_white"].mean().item())
        if "dist_yellow" in infos:
            self.track_data("Telemetry / Dist_Yellow_m", infos["dist_yellow"].mean().item())
        if "track_progress_pct" in infos:
            track_pct = infos["track_progress_pct"].mean().item()
            self.track_data("Telemetry / Track_Progress_Pct", track_pct)
        if "track_wp_delta" in infos:
            # Cumulative WPs from spawn this episode (obs-layer variable)
            wp_delta = infos["track_wp_delta"].float().mean().item()
            self.track_data("Telemetry / WP_Delta_Cumulative", wp_delta)
        # FIX B: Log the ACTUAL per-step reward delta (rewards.py variable)
        wp_step = 0.0
        if "reward_wp_delta_step" in infos:
            wp_step = infos["reward_wp_delta_step"].float().mean().item()
            self.track_data("Telemetry / WP_Delta_Step_Reward", wp_step)
            
        # Add custom clean text logging
        self._print_counter += states.shape[0] if states is not None else 1
        if self._print_counter >= 10000:
            ep_len_list = self.tracking_data.get("Episode / Total timesteps (mean)", [])
            ep_rew_list = self.tracking_data.get("Reward / Total reward (mean)", [])
            
            if len(ep_len_list) > 0 and ep_len_list[-1] != 0:
                self._last_ep_len = ep_len_list[-1]
            if len(ep_rew_list) > 0 and ep_rew_list[-1] != 0:
                self._last_ep_rew = ep_rew_list[-1]
                
            # FIX A: Trk% is always 0.0% with 149k WPs — useless. Log raw cumulative WP count
            # and the per-step reward delta instead.
            # Trk% = 0.01% needs 1499 WPs traversed (9m of track), rounds to 0.0 with :.1f
            print(f"Step {timestep} | Rew: {self._last_ep_rew:.1f} | Len: {self._last_ep_len:.0f} | "
                  f"Spd: {speed:.2f} | WPs_cum: {wp_delta:.0f} | WPΔ_rew: {wp_step:.2f}")
            sys.stdout.flush()
            self._print_counter = 0

class WarmupActionWrapper(gym.Wrapper):
    def __init__(self, env, warmup_steps=10):
        super().__init__(env)
        self.warmup_steps = warmup_steps
        
    def step(self, action):
        if hasattr(self.env.unwrapped, "episode_length_buf"):
            mask = self.env.unwrapped.episode_length_buf < self.warmup_steps
            action = action.clone()
            action[mask, 0] = 0.0   # Steer: 0.0 is center
            action[mask, 1] = -1.0  # Throttle: action * (-20) - 20 = 0.0 (Stop)
        return self.env.step(action)

def main():
    # 1. Setup Environment Configuration
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.enable_cameras = args_cli.enable_cameras 
    env_cfg.__post_init__() 

    # 2. Create Environment
    env = ManagerBasedRLEnv(cfg=env_cfg)
    env = WarmupActionWrapper(env, warmup_steps=10)
    
    # 3. Wrap for SKRL
    # The IsaacLabWrapper automatically extracts the "critic" observation group
    # (if it exists) into env.state_space and env.state(), enabling AAC.
    env = IsaacLabWrapper(env)
    
    # 3.5 Flatten for SKRL RandomMemory limitation
    env = SKRLFlattenWrapper(env)
    
    # 4. Define Memory (reduce size to prevent OOM)
    memory = RandomMemory(memory_size=128, num_envs=env.num_envs, device="cuda:0")
    
    # 5. Define Models
    models = {}
    models["policy"] = ARCProActor(env.observation_space, env.action_space, "cuda:0")
    
    # Use env.state_space for the Critic if available, fallback to observation_space
    state_space = getattr(env, "state_space", env.observation_space)
    models["value"] = ARCProCritic(state_space, env.action_space, "cuda:0")
    
    # 6. Configure Agent
    cfg_ppo = PPO_DEFAULT_CONFIG.copy()
    cfg_ppo["rollouts"] = 128
    cfg_ppo["mini_batches"] = 16  # Small mini-batch size (128*4/16 = 32 images per backprop)
    cfg_ppo["learning_rate"] = 1e-4
    cfg_ppo["random_timesteps"] = 0
    cfg_ppo["learning_starts"] = 0
    cfg_ppo["state_preprocessor"] = None
    # Use native PPO KL early stopping and Entropy to prevent policy collapse
    cfg_ppo["kl_threshold"] = 0.008
    cfg_ppo["entropy_loss_scale"] = 0.001
    cfg_ppo["learning_rate_scheduler"] = None
    
    log_dir = os.path.join("logs", "ppo_skrl", datetime.now().strftime("%Y%m%d-%H%M%S"))
    cfg_ppo["experiment"]["directory"] = log_dir
    cfg_ppo["experiment"]["write_interval"] = 100
    cfg_ppo["experiment"]["checkpoint_interval"] = 250  # Save every 250 rollouts (approx 32k steps)
    
    agent = TelemetryPPO(models=models, memory=memory, cfg=cfg_ppo, observation_space=env.observation_space, action_space=env.action_space, device="cuda:0")
    
    if args_cli.resume:
        print(f"Resuming from checkpoint: {args_cli.resume}")
        agent.load(args_cli.resume)
        
    # 7. Train
    print(f"Starting SKRL AAC training for {args_cli.total_timesteps} steps...")
    trainer_cfg = {
        "timesteps": args_cli.total_timesteps,
        "disable_progressbar": True
    }
    trainer = SequentialTrainer(cfg=trainer_cfg, env=env, agents=agent)
    trainer.train()

    env.close()

if __name__ == "__main__":
    main()
