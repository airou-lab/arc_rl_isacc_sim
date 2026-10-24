import gc

import gymnasium as gym
import numpy as np
import torch
import torch.nn as nn
from skrl.envs.wrappers.torch import Wrapper
from torchvision.models import ResNet18_Weights, resnet18


class SKRLFlattenWrapper(Wrapper):
    def __init__(self, env):
        super().__init__(env)
        self._step_counter = 0
        if self._env.cfg.enable_cameras:
            # 1. Fused YUV Linear Preprocessing Layer (ITU-R BT.601)
            # Isolates luminance (white road boundaries) on Y, and chrominance (yellow median) on U, V
            self.yuv_conv = nn.Conv2d(3, 3, kernel_size=1, bias=False).to("cuda:0")
            yuv_weights = (
                torch.tensor(
                    [
                        [0.299, 0.587, 0.114],
                        [-0.14713, -0.28886, 0.436],
                        [0.615, -0.51499, -0.10001],
                    ],
                    dtype=torch.float32,
                )
                .unsqueeze(-1)
                .unsqueeze(-1)
            )
            self.yuv_conv.weight.data.copy_(yuv_weights)
            self.yuv_conv.weight.requires_grad = False
            self.yuv_conv.eval()

            # YUV normalization: Y in [0, 1] centered at 0.5, U/V in [-0.5, 0.5] centered at 0.0
            self.yuv_mean = torch.tensor([0.5, 0.0, 0.0]).view(1, 3, 1, 1).to("cuda:0")
            self.yuv_std = torch.tensor([0.5, 0.5, 0.5]).view(1, 3, 1, 1).to("cuda:0")

            base = resnet18(weights=ResNet18_Weights.IMAGENET1K_V1).to("cuda:0")
            for param in base.parameters():
                param.requires_grad = False
            self.resnet_conv = nn.Sequential(
                base.conv1,
                base.bn1,
                base.relu,
                base.maxpool,
                base.layer1,
                base.layer2,
                base.layer3,
                base.layer4,
            ).to("cuda:0")
            self.resnet_conv.eval()

    @property
    def observation_space(self):
        # 12 telemetry + 25088 (512x7x7 2D spatial feature map) if camera is enabled
        shape_dim = 25100 if self._env.cfg.enable_cameras else 12
        return gym.spaces.Box(low=-np.inf, high=np.inf, shape=(shape_dim,))

    def step(self, actions):
        obs, reward, terminated, truncated, info = self._env.step(actions)

        try:
            base_env = self._env.unwrapped
            robot_asset = base_env.scene["robot"]
            speed = torch.norm(robot_asset.data.root_lin_vel_b[:, :2], dim=1).detach()
            info["speed_mps"] = speed

            if "dist_w" in base_env.extras:
                info["dist_white"] = base_env.extras["dist_w"].detach()
            if "dist_y" in base_env.extras:
                info["dist_yellow"] = base_env.extras["dist_y"].detach()
            if "track_progress_pct" in base_env.extras:
                info["track_progress_pct"] = base_env.extras[
                    "track_progress_pct"
                ].detach()
            if "track_wp_delta" in base_env.extras:
                info["track_wp_delta"] = base_env.extras["track_wp_delta"].detach()
            if "reward_wp_delta_step" in base_env.extras:
                info["reward_wp_delta_step"] = base_env.extras[
                    "reward_wp_delta_step"
                ].detach()
            if "lat_err" in base_env.extras:
                info["lat_err"] = base_env.extras["lat_err"].detach()
            if "head_err" in base_env.extras:
                info["head_err"] = base_env.extras["head_err"].detach()
        except Exception:
            pass

        flat_obs = self._flatten_obs(obs)
        self._step_counter += 1
        if self._step_counter % 50 == 0:
            gc.collect()
            torch.cuda.empty_cache()

        return flat_obs, reward, terminated, truncated, info

    def reset(self):
        obs, info = self._env.reset()
        try:
            base_env = self._env.unwrapped
            if "dist_w" in base_env.extras:
                info["dist_white"] = base_env.extras["dist_w"].detach()
            if "dist_y" in base_env.extras:
                info["dist_yellow"] = base_env.extras["dist_y"].detach()
            if "lat_err" in base_env.extras:
                info["lat_err"] = base_env.extras["lat_err"].detach()
            if "head_err" in base_env.extras:
                info["head_err"] = base_env.extras["head_err"].detach()
        except Exception:
            pass
        return self._flatten_obs(obs), info

    def _flatten_obs(self, obs):
        if isinstance(obs, dict):
            if "telemetry" in obs:
                vec = obs["telemetry"].detach()
            elif "policy" in obs:
                p = obs["policy"]
                vec = p["telemetry"].detach() if isinstance(p, dict) else p.detach()
            else:
                vec = list(obs.values())[0].detach()
        else:
            vec = obs.detach()

        if self._env.cfg.enable_cameras:
            with torch.no_grad():
                base_env = self._env.unwrapped
                cam_sensor = (
                    base_env.scene.sensors.get("tiled_camera")
                    if hasattr(base_env.scene, "sensors")
                    else None
                )
                if cam_sensor is not None and "rgb" in cam_sensor.data.output:
                    cam_tensor = cam_sensor.data.output["rgb"]
                elif isinstance(obs, dict) and "tiled_camera" in obs:
                    cam_tensor = obs.pop("tiled_camera")
                else:
                    return vec

                img = cam_tensor.float()
                # Permute from NHWC to NCHW
                if img.dim() == 4 and img.shape[-1] == 3:
                    img = img.permute(0, 3, 1, 2)
                if img.max() > 1.0:
                    img = img / 255.0
                # Convert RGB to YUV and normalize
                yuv = self.yuv_conv(img)
                norm_img = (yuv - self.yuv_mean) / self.yuv_std
                feat_map = self.resnet_conv(norm_img)  # (B, 512, 7, 7)
                visual_flat = feat_map.flatten(start_dim=1)  # (B, 25088)

                num_envs = vec.shape[0]
                total_dim = 25088 + vec.shape[1]
                if (
                    not hasattr(self, "_flat_obs_buffers")
                    or self._flat_obs_buffers is None
                    or self._flat_obs_buffers[0].shape != (num_envs, total_dim)
                ):
                    self._flat_obs_buffers = [
                        torch.zeros(
                            (num_envs, total_dim),
                            device=vec.device,
                            dtype=torch.float32,
                        ),
                        torch.zeros(
                            (num_envs, total_dim),
                            device=vec.device,
                            dtype=torch.float32,
                        ),
                    ]
                    self._buf_idx = 0

                self._buf_idx = (self._buf_idx + 1) % 2
                buf = self._flat_obs_buffers[self._buf_idx]
                buf[:, :25088].copy_(visual_flat)
                buf[:, 25088:].copy_(vec)
                self._flat_obs_buffer = buf
                del img, yuv, norm_img, feat_map, visual_flat
                return buf

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


class WarmupActionWrapper(gym.Wrapper):
    def __init__(self, env, warmup_steps=10):
        super().__init__(env)
        self.warmup_steps = warmup_steps

    def step(self, action):
        if hasattr(self.env.unwrapped, "episode_length_buf"):
            mask = self.env.unwrapped.episode_length_buf < self.warmup_steps
            action = action.clone()
            action[mask, 0] = 0.0  # Steer: 0.0 is center
            action[mask, 1] = -1.0  # Throttle: action * (-20) - 20 = 0.0 (Stop)
        return self.env.step(action)
