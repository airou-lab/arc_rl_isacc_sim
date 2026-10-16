import torch
import torch.nn as nn
from skrl.envs.wrappers.torch import Wrapper
import numpy as np
import gymnasium as gym
from torchvision.models import resnet18, ResNet18_Weights
import gc


class SKRLFlattenWrapper(Wrapper):
    def __init__(self, env):
        super().__init__(env)
        self._step_counter = 0
        if self._env.cfg.enable_cameras:
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
            self.mean = (
                torch.tensor([0.485, 0.456, 0.406]).view(1, 3, 1, 1).to("cuda:0")
            )
            self.std = torch.tensor([0.229, 0.224, 0.225]).view(1, 3, 1, 1).to("cuda:0")

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
                img = (img - self.mean) / self.std
                feat_map = self.resnet_conv(img)  # (B, 512, 7, 7)
                visual_flat = feat_map.flatten(start_dim=1)  # (B, 25088)

                num_envs = vec.shape[0]
                total_dim = 25088 + vec.shape[1]
                if (
                    not hasattr(self, "_flat_obs_buffer")
                    or self._flat_obs_buffer is None
                    or self._flat_obs_buffer.shape != (num_envs, total_dim)
                ):
                    self._flat_obs_buffer = torch.zeros(
                        (num_envs, total_dim), device=vec.device, dtype=torch.float32
                    )

                self._flat_obs_buffer[:, :25088].copy_(visual_flat)
                self._flat_obs_buffer[:, 25088:].copy_(vec)
                del img, feat_map, visual_flat
                return self._flat_obs_buffer

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
