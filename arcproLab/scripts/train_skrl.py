import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Train SKRL AAC policy for ARCPro Lane Following.")
parser.add_argument("--num_envs", type=int, default=16, help="Number of parallel simulation environments.")
parser.add_argument("--seed", type=int, default=42, help="Seed for the environment.")
parser.add_argument("--total_timesteps", type=int, default=5000000, help="Total timesteps to train.")
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

class SKRLFlattenWrapper(Wrapper):
    def __init__(self, env):
        super().__init__(env)
        if self._env.cfg.enable_cameras:
            self.resnet = resnet18(weights=ResNet18_Weights.IMAGENET1K_V1).to("cuda:0")
            self.resnet.eval()
            for param in self.resnet.parameters():
                param.requires_grad = False
            self.resnet.fc = nn.Identity()
            self.register_buffer = lambda name, tensor: setattr(self, name, tensor)
            self.mean = torch.tensor([0.485, 0.456, 0.406]).view(1, 3, 1, 1).to("cuda:0")
            self.std = torch.tensor([0.229, 0.224, 0.225]).view(1, 3, 1, 1).to("cuda:0")
        
    @property
    def observation_space(self):
        # 12 telemetry + 512 vision if camera is enabled
        shape_dim = 524 if self._env.cfg.enable_cameras else 12
        return gym.spaces.Box(low=-np.inf, high=np.inf, shape=(shape_dim,))
        
    def step(self, actions):
        obs, reward, terminated, truncated, info = self._env.step(actions)
        
        # Inject telemetry into info for SKRL logger
        try:
            base_env = self._env.unwrapped
            robot_asset = base_env.scene["robot"]
            speed = torch.norm(robot_asset.data.root_lin_vel_b[:, :2], dim=1)
            info["speed_mps"] = speed.clone()
            
            # Extract boundary distances if available in extras
            if "dist_w" in base_env.extras:
                info["dist_white"] = base_env.extras["dist_w"].clone()
            if "dist_y" in base_env.extras:
                info["dist_yellow"] = base_env.extras["dist_y"].clone()
            # Track progress telemetry
            if "track_progress_pct" in base_env.extras:
                info["track_progress_pct"] = base_env.extras["track_progress_pct"].clone()
            if "track_wp_delta" in base_env.extras:
                info["track_wp_delta"] = base_env.extras["track_wp_delta"].clone()
            # FIX B: Expose the per-step reward WP delta (from rewards.py, not obs.py)
            if "reward_wp_delta_step" in base_env.extras:
                info["reward_wp_delta_step"] = base_env.extras["reward_wp_delta_step"].clone()
        except Exception:
            pass
            
        return self._flatten_obs(obs), reward, terminated, truncated, info
        
    def reset(self):
        obs, info = self._env.reset()
        return self._flatten_obs(obs), info
        
    def _flatten_obs(self, obs):
        vec = obs["telemetry"]
        if "tiled_camera" in obs:
            img = obs["tiled_camera"].float()
            if img.dim() == 4 and img.shape[-1] == 3:
                img = img.permute(0, 3, 1, 2)
            if img.max() > 1.0:
                img = img / 255.0
            img = (img - self.mean) / self.std
            with torch.no_grad():
                visual_feats = self.resnet(img)
            return torch.cat([visual_feats, vec], dim=1)
        return vec
        
    @property
    def state_space(self):
        return self._env.state_space
        
    @property
    def action_space(self):
        return self._env.action_space
        
    def state(self):
        return self._env.state()
        
    def render(self, *args, **kwargs):
        return self._env.render(*args, **kwargs)
        
    def close(self):
        self._env.close()

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

def main():
    # 1. Setup Environment Configuration
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.enable_cameras = args_cli.enable_cameras 
    env_cfg.__post_init__() 

    # 2. Create Environment
    env = ManagerBasedRLEnv(cfg=env_cfg)
    
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
    cfg_ppo["value_preprocessor"] = None
    log_dir = os.path.join("logs", "ppo_skrl", datetime.now().strftime("%Y%m%d-%H%M%S"))
    cfg_ppo["experiment"]["directory"] = log_dir
    cfg_ppo["experiment"]["write_interval"] = 100
    cfg_ppo["experiment"]["checkpoint_interval"] = max(1, 500000 // env.num_envs)
    
    agent = TelemetryPPO(models=models, memory=memory, cfg=cfg_ppo, observation_space=env.observation_space, action_space=env.action_space, device="cuda:0")
    
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
