from __future__ import annotations

import gc

import gymnasium as gym
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from skrl.envs.wrappers.torch import Wrapper

try:
    from mdp.track_topology import (
        TELEMETRY_DIM,
        TELEMETRY_MIRROR_SIGNS,
        ACTION_MIRROR_SIGNS,
        ACTOR_MASKED_TELEMETRY_INDICES,
    )
except ImportError:
    try:
        from arcproLab.mdp.track_topology import (
            TELEMETRY_DIM,
            TELEMETRY_MIRROR_SIGNS,
            ACTION_MIRROR_SIGNS,
            ACTOR_MASKED_TELEMETRY_INDICES,
        )
    except ImportError:
        TELEMETRY_DIM = 12
        TELEMETRY_MIRROR_SIGNS = (
            -1.0,
            1.0,
            1.0,
            1.0,
            -1.0,
            -1.0,
            1.0,
            -1.0,
            -1.0,
            -1.0,
            -1.0,
            1.0,
        )
        ACTION_MIRROR_SIGNS = (-1.0, 1.0)
        ACTOR_MASKED_TELEMETRY_INDICES = (5, 6, 8, 9, 11)

try:
    from agents.skrl_models import (
        BT601_YUV_MATRIX,
        IMAGENET_MEAN,
        IMAGENET_STD,
        YUV_MEAN,
        YUV_STD,
        load_resnet18_backbone,
        preprocess_camera_tensor,
    )
except ImportError:
    from arcproLab.agents.skrl_models import (
        BT601_YUV_MATRIX,
        IMAGENET_MEAN,
        IMAGENET_STD,
        YUV_MEAN,
        YUV_STD,
        load_resnet18_backbone,
        preprocess_camera_tensor,
    )


class SKRLFlattenWrapper(Wrapper):
    def __init__(
        self,
        env,
        vision_mode: str = "rgb",
        training: bool = True,
        equivariant: bool = True,
        augment: bool | None = None,
    ):
        super().__init__(env)
        self._step_counter = 0
        self.vision_mode = vision_mode.lower()
        self.training = training if augment is None else (bool(training) and bool(augment))
        self.augment = self.training
        self.equivariant = equivariant

        raw_dev = getattr(
            self._env,
            "device",
            getattr(
                self._env.unwrapped,
                "device",
                "cuda:0" if torch.cuda.is_available() else "cpu",
            ),
        )
        self._device = torch.device(raw_dev)

        if self._env.cfg.enable_cameras:
            # 1. Legacy YUV Linear Preprocessing Layer (ITU-R BT.601 compatibility)
            self.yuv_conv = nn.Conv2d(3, 3, kernel_size=1, bias=False).to(self._device)
            yuv_weights = (
                torch.tensor(BT601_YUV_MATRIX, dtype=torch.float32)
                .unsqueeze(-1)
                .unsqueeze(-1)
            )
            self.yuv_conv.weight.data.copy_(yuv_weights)
            self.yuv_conv.weight.requires_grad = False
            self.yuv_conv.eval()

            self.yuv_mean = (
                torch.tensor(YUV_MEAN, dtype=torch.float32, device=self._device).view(
                    1, 3, 1, 1
                )
            )
            self.yuv_std = (
                torch.tensor(YUV_STD, dtype=torch.float32, device=self._device).view(
                    1, 3, 1, 1
                )
            )
            self.rgb_mean = (
                torch.tensor(
                    IMAGENET_MEAN, dtype=torch.float32, device=self._device
                ).view(1, 3, 1, 1)
            )
            self.rgb_std = (
                torch.tensor(
                    IMAGENET_STD, dtype=torch.float32, device=self._device
                ).view(1, 3, 1, 1)
            )

            self.resnet_conv = load_resnet18_backbone(
                device=self._device, pretrained=True
            )

    @property
    def observation_space(self):
        # 12 telemetry + 25088 (512x7x7 2D spatial feature map) if camera is enabled
        shape_dim = 25088 + TELEMETRY_DIM if self._env.cfg.enable_cameras else TELEMETRY_DIM
        return gym.spaces.Box(low=-np.inf, high=np.inf, shape=(shape_dim,))

    def _apply_visual_domain_randomization(self, img: torch.Tensor) -> torch.Tensor:
        """Apply photometric domain randomization in [0, 1] RGB without horizontal flipping."""
        B = img.shape[0]
        dev = img.device
        dtype = img.dtype

        brightness = (
            torch.rand((B, 1, 1, 1), device=dev, dtype=dtype) * 2.0 - 1.0
        ) * 0.12
        contrast = 0.85 + torch.rand((B, 1, 1, 1), device=dev, dtype=dtype) * 0.30
        gamma = 0.90 + torch.rand((B, 1, 1, 1), device=dev, dtype=dtype) * 0.20

        img = torch.clamp(img, 1e-6, 1.0).pow(gamma)
        mean_lum = img.mean(dim=(-3, -2, -1), keepdim=True)
        img = (img - mean_lum) * contrast + mean_lum + brightness
        img = img + torch.randn_like(img) * 0.02
        return torch.clamp(img, 0.0, 1.0)

    def _preprocess_and_encode(self, cam_tensor: torch.Tensor) -> torch.Tensor:
        if cam_tensor.dtype == torch.uint8:
            img = cam_tensor.to(device=self._device, dtype=torch.float32) * (
                1.0 / 255.0
            )
        else:
            img = cam_tensor.to(device=self._device, dtype=torch.float32)
            max_per_sample = img.amax(dim=(-3, -2, -1), keepdim=True)
            scale = torch.where(
                max_per_sample > 1.5,
                torch.tensor(1.0 / 255.0, device=img.device, dtype=img.dtype),
                torch.tensor(1.0, device=img.device, dtype=img.dtype),
            )
            img = img * scale

        if img.dim() == 4 and img.shape[1] != 3 and img.shape[-1] == 3:
            img = img.permute(0, 3, 1, 2).contiguous()

        if img.shape[-2] != 224 or img.shape[-1] != 224:
            img = F.interpolate(
                img, size=(224, 224), mode="bilinear", align_corners=False
            )

        if self.training and self.vision_mode == "rgb":
            img = self._apply_visual_domain_randomization(img)

        if self.vision_mode == "rgb":
            norm_img = (img - self.rgb_mean) / self.rgb_std
        elif self.vision_mode == "yuv":
            yuv = self.yuv_conv(img)
            norm_img = (yuv - self.yuv_mean) / self.yuv_std
        else:
            norm_img = preprocess_camera_tensor(img, vision_mode=self.vision_mode)

        if self.equivariant:
            img_pair = torch.cat([norm_img, torch.flip(norm_img, dims=[-1])], dim=0)
            feat_pair = self.resnet_conv(img_pair)
            feat_pos, feat_neg = feat_pair.chunk(2, dim=0)
            feat_map = 0.5 * (feat_pos + torch.flip(feat_neg, dims=[-1]))
        else:
            feat_map = self.resnet_conv(norm_img)

        return feat_map.flatten(start_dim=1)

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
            if torch.cuda.is_available():
                torch.cuda.empty_cache()

        return flat_obs, reward, terminated, truncated, info

    def reset(self):
        if hasattr(self._env, "_reset_once"):
            self._env._reset_once = True
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

                visual_flat = self._preprocess_and_encode(cam_tensor)

                num_envs = vec.shape[0]
                total_dim = 25088 + vec.shape[1]
                if (
                    not hasattr(self, "_flat_obs_buffers")
                    or self._flat_obs_buffers is None
                    or self._flat_obs_buffers[0].shape != (num_envs, total_dim)
                    or self._flat_obs_buffers[0].device != vec.device
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
                buf[:, :25088].copy_(visual_flat.to(device=vec.device))
                buf[:, 25088:].copy_(vec)
                self._flat_obs_buffer = buf
                del visual_flat
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
    """Pass-through wrapper preserving API compatibility without off-policy action substitution.

    Previously, overwriting actions inside step() after agent.act() poisoned RandomMemory with
    (s_t, a_policy, log_prob(a_policy), r_warmup, s_{t+1}_warmup) off-policy transitions.
    env.step() now executes the exact action tensor recorded in memory.
    """

    def __init__(self, env, warmup_steps: int = 0):
        super().__init__(env)
        self.warmup_steps = warmup_steps

    def step(self, action):
        return self.env.step(action)
