import torch
import torch.nn as nn
from skrl.envs.wrappers.torch import Wrapper
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
            # Expose the per-step reward WP delta
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
            # Permute from NHWC to NCHW
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
