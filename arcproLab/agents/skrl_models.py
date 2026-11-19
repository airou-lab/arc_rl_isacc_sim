from __future__ import annotations

import os
from typing import Any

import torch
import torch.nn as nn
import torch.nn.functional as F
from packaging import version
import skrl
from skrl.models.torch import DeterministicMixin, GaussianMixin, Model
from torchvision.models import ResNet18_Weights, resnet18

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

_SKRL_V2 = version.parse(getattr(skrl, "__version__", "1.0.0")) >= version.parse("2.0.0")

# Ensure skrl.agents.torch.ppo exposes PPO_DEFAULT_CONFIG even under skrl >= 2.0
try:
    import skrl.agents.torch.ppo as _skrl_ppo_mod

    if not hasattr(_skrl_ppo_mod, "PPO_DEFAULT_CONFIG"):
        _skrl_ppo_mod.PPO_DEFAULT_CONFIG = {
            "rollouts": 16,
            "learning_epochs": 8,
            "mini_batches": 2,
            "discount_factor": 0.99,
            "lambda": 0.95,
            "learning_rate": 1e-3,
            "learning_rate_scheduler": None,
            "learning_rate_scheduler_kwargs": {},
            "state_preprocessor": None,
            "state_preprocessor_kwargs": {},
            "value_preprocessor": None,
            "value_preprocessor_kwargs": {},
            "random_timesteps": 0,
            "learning_starts": 0,
            "grad_norm_clip": 0.5,
            "ratio_clip": 0.2,
            "value_clip": 0.2,
            "clip_predicted_values": False,
            "entropy_loss_scale": 0.0,
            "value_loss_scale": 1.0,
            "kl_threshold": 0.0,
            "rewards_shaper": None,
            "time_limit_bootstrap": False,
            "experiment": {
                "directory": "",
                "experiment_name": "",
                "write_interval": "auto",
                "checkpoint_interval": "auto",
                "store_separately": False,
                "wandb": False,
                "wandb_kwargs": {},
            },
        }
except Exception:
    pass


VISUAL_FEATURE_DIM = 25088
NUM_KEYPOINTS = 32
IMAGENET_MEAN = (0.485, 0.456, 0.406)
IMAGENET_STD = (0.229, 0.224, 0.225)
YUV_MEAN = (0.5, 0.0, 0.0)
YUV_STD = (0.5, 0.5, 0.5)
BT601_YUV_MATRIX = (
    (0.299, 0.587, 0.114),
    (-0.14713, -0.28886, 0.436),
    (0.615, -0.51499, -0.10001),
)
RESNET18_CACHE_PATH = os.path.expanduser(
    "~/.cache/torch/hub/checkpoints/resnet18-f37072fd.pth"
)


def load_resnet18_backbone(
    device: str | torch.device = "cpu", pretrained: bool = True
) -> nn.Sequential:
    """Load ResNet-18 conv1..layer4 backbone safely from local torch cache without network downloads."""
    base = resnet18(weights=None)
    if pretrained:
        if os.path.isfile(RESNET18_CACHE_PATH):
            try:
                sd = torch.load(
                    RESNET18_CACHE_PATH, map_location="cpu", weights_only=True
                )
            except TypeError:
                sd = torch.load(RESNET18_CACHE_PATH, map_location="cpu")
            base.load_state_dict(sd, strict=False)
        else:
            try:
                base = resnet18(weights=ResNet18_Weights.IMAGENET1K_V1)
            except Exception:
                pass

    for param in base.parameters():
        param.requires_grad = False

    backbone = nn.Sequential(
        base.conv1,
        base.bn1,
        base.relu,
        base.maxpool,
        base.layer1,
        base.layer2,
        base.layer3,
        base.layer4,
    ).to(device)
    backbone.eval()
    return backbone


def preprocess_camera_tensor(
    rgb: torch.Tensor, vision_mode: str = "rgb"
) -> torch.Tensor:
    """Normalize camera RGB tensor into (B, 3, 224, 224) float32 tensor without GPU-CPU syncs.

    Accepts (B, H, W, 3) or (B, 3, H, W) (or 3D unbatched) in uint8 [0, 255] or float [0, 255] / [0, 1].
    Supports vision_mode="rgb" (ImageNet mean/std) and vision_mode="yuv" (ITU-R BT.601 YUV).
    """
    was_3d = rgb.dim() == 3
    if was_3d:
        rgb = rgb.unsqueeze(0)

    if rgb.dtype == torch.uint8:
        img = rgb.to(dtype=torch.float32) * (1.0 / 255.0)
    else:
        img = rgb.to(dtype=torch.float32)
        # Pure device-side scaling check: zero GPU-CPU .item() synchronization
        max_per_sample = img.amax(dim=(-3, -2, -1), keepdim=True)
        scale = torch.where(
            max_per_sample > 1.5,
            torch.tensor(1.0 / 255.0, device=img.device, dtype=img.dtype),
            torch.tensor(1.0, device=img.device, dtype=img.dtype),
        )
        img = img * scale

    # Permute NHWC -> NCHW if needed
    if img.dim() == 4 and img.shape[1] != 3 and img.shape[-1] == 3:
        img = img.permute(0, 3, 1, 2).contiguous()

    if img.shape[-2] != 224 or img.shape[-1] != 224:
        img = F.interpolate(img, size=(224, 224), mode="bilinear", align_corners=False)

    mode = vision_mode.lower()
    if mode == "rgb":
        mean = torch.tensor(IMAGENET_MEAN, device=img.device, dtype=img.dtype).view(
            1, 3, 1, 1
        )
        std = torch.tensor(IMAGENET_STD, device=img.device, dtype=img.dtype).view(
            1, 3, 1, 1
        )
        return (img - mean) / std
    elif mode == "yuv":
        yuv_mat = torch.tensor(
            BT601_YUV_MATRIX, device=img.device, dtype=img.dtype
        )
        yuv = torch.einsum("cd,bdhw->bchw", yuv_mat, img)
        mean = torch.tensor(YUV_MEAN, device=img.device, dtype=img.dtype).view(
            1, 3, 1, 1
        )
        std = torch.tensor(YUV_STD, device=img.device, dtype=img.dtype).view(
            1, 3, 1, 1
        )
        return (yuv - mean) / std
    elif mode == "raw":
        return img
    else:
        raise ValueError(f"Unsupported vision_mode '{vision_mode}'. Expected 'rgb' or 'yuv'.")


def synthesize_road_image(
    batch_size: int | str = 1,
    height: int = 224,
    width: int = 224,
    scenario: str = "centered_sym",
    vision_mode: str = "rgb",
    device: str | torch.device = "cpu",
    lateral_offset: float = 0.0,
    heading_error: float = 0.0,
) -> torch.Tensor:
    """Synthesize deterministic geometric road scenes for real-image visual probes and tests.

    Supported scenarios:
    - "centered_sym" (also "asphalt_sym", "white_L_white_R"): 100% L/R pixel-symmetric road.
    - "yellow_L_white_R": Yellow left median, white right boundary.
    - "white_L_yellow_R" (also "white_L_yellow_R_mirror"): Exact horizontal mirror of "yellow_L_white_R".
    - "left_turn": Left-curving lane lines.
    - "right_turn": Exact horizontal mirror of "left_turn".
    """
    if isinstance(batch_size, str):
        scenario = batch_size
        batch_size = 1
    dev = torch.device(device)
    y = torch.linspace(0.0, 1.0, height, device=dev, dtype=torch.float32).view(
        1, 1, height, 1
    )
    # Antisymmetric x grid: x.flip(-1) == -x
    x = torch.linspace(-1.0, 1.0, width, device=dev, dtype=torch.float32).view(
        1, 1, 1, width
    )

    horizon = 0.35
    is_road = (y >= horizon).to(dtype=torch.float32)
    t = torch.clamp((y - horizon) / (1.0 - horizon), 0.0, 1.0)

    # Base sky + perspective-shaded asphalt (100% L/R symmetric)
    asphalt_val = 0.22 + 0.08 * t
    r = (1.0 - is_road) * 0.18 + is_road * asphalt_val
    g = (1.0 - is_road) * 0.20 + is_road * asphalt_val
    b = (1.0 - is_road) * 0.25 + is_road * asphalt_val
    img = torch.cat(
        [
            r.expand(1, 1, height, width),
            g.expand(1, 1, height, width),
            b.expand(1, 1, height, width),
        ],
        dim=1,
    ).clone()

    scen = scenario.lower()
    curve_offset = float(lateral_offset) + float(heading_error) * (1.0 - t)
    if scen == "left_turn":
        curve_offset = curve_offset - 0.28 * ((1.0 - t) ** 2)
    elif scen == "right_turn":
        curve_offset = curve_offset + 0.28 * ((1.0 - t) ** 2)

    half_span = 0.16 + 0.52 * t
    x_left = -half_span + curve_offset
    x_right = +half_span + curve_offset
    line_half_width = 0.025 + 0.035 * t

    # Smooth antialiased line masks
    left_mask = (
        is_road
        * torch.sigmoid((line_half_width - (x - x_left).abs()) * 80.0)
    )
    right_mask = (
        is_road
        * torch.sigmoid((line_half_width - (x - x_right).abs()) * 80.0)
    )

    white_rgb = torch.tensor([0.92, 0.92, 0.92], device=dev, dtype=torch.float32).view(
        1, 3, 1, 1
    )
    yellow_rgb = torch.tensor([0.95, 0.78, 0.08], device=dev, dtype=torch.float32).view(
        1, 3, 1, 1
    )

    if scen in ("centered_sym", "white_l_white_r", "asphalt_sym"):
        left_color = white_rgb
        right_color = white_rgb
    elif scen == "yellow_l_white_r":
        left_color = yellow_rgb
        right_color = white_rgb
    elif scen in ("white_l_yellow_r", "white_l_yellow_r_mirror"):
        left_color = white_rgb
        right_color = yellow_rgb
    elif scen == "left_turn":
        left_color = yellow_rgb
        right_color = white_rgb
    elif scen == "right_turn":
        left_color = white_rgb
        right_color = yellow_rgb
    else:
        raise ValueError(
            f"Unknown road scenario '{scenario}'. Supported: "
            "'centered_sym', 'yellow_L_white_R', 'white_L_yellow_R', 'left_turn', 'right_turn'."
        )

    img = img * (1.0 - left_mask) + left_color * left_mask
    img = img * (1.0 - right_mask) + right_color * right_mask
    img = torch.clamp(img, 0.0, 1.0).expand(batch_size, 3, height, width).contiguous()

    return preprocess_camera_tensor(img, vision_mode=vision_mode)


class SpatialSoftmax2d(nn.Module):
    """
    2D Spatial Softmax (Keypoint Extraction) Module.
    Grounded in Levine et al. (2016), JMLR.
    Transforms spatial feature maps (B, C, H, W) into continuous 2D coordinates (B, 2*C) in [-1, 1].
    """

    def __init__(self, height: int = 7, width: int = 7, temperature: float = 1.0):
        super().__init__()
        self.height = height
        self.width = width
        self.temperature = temperature

        pos_y, pos_x = torch.meshgrid(
            torch.linspace(-1.0, 1.0, height, dtype=torch.float32),
            torch.linspace(-1.0, 1.0, width, dtype=torch.float32),
            indexing="ij",
        )
        self.register_buffer("pos_x", pos_x.reshape(-1))
        self.register_buffer("pos_y", pos_y.reshape(-1))

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        # x: (B, C, H, W)
        B, C, H, W = x.shape
        flat_x = x.reshape(B, C, H * W) / self.temperature
        softmax_attention = torch.softmax(flat_x, dim=-1)

        pos_x = self.pos_x.to(dtype=x.dtype, device=x.device)
        pos_y = self.pos_y.to(dtype=x.dtype, device=x.device)

        expected_x = torch.sum(softmax_attention * pos_x, dim=-1)  # (B, C)
        expected_y = torch.sum(softmax_attention * pos_y, dim=-1)  # (B, C)

        # Interleave (x, y) coordinates: (B, 2*C)
        coords = torch.stack([expected_x, expected_y], dim=-1).view(B, 2 * C)
        return coords


class ARCProActor(GaussianMixin, Model):
    def __init__(
        self,
        observation_space,
        action_space,
        device,
        clip_actions: bool = False,
        clip_log_std: bool = False,
        min_log_std: float = -2.0,
        max_log_std: float = 0.5,
        reduction: str = "sum",
        equivariant: bool = True,
        vision_mode: str = "rgb",
    ):
        Model.__init__(
            self,
            observation_space=observation_space,
            action_space=action_space,
            device=device,
        )
        # Disable hard torch.clamp in GaussianMixin so smooth bijection in get_log_std()
        # preserves strictly positive gradient d(log_std)/d(log_std_parameter) > 0 everywhere.
        GaussianMixin.__init__(
            self,
            clip_actions=clip_actions,
            clip_log_std=False,
            min_log_std=min_log_std,
            max_log_std=max_log_std,
            reduction=reduction,
        )

        self.equivariant = equivariant
        self.vision_mode = vision_mode
        self.num_visual_features = 25088  # 512 channels x 7 x 7 spatial grid
        self.num_telemetry = TELEMETRY_DIM
        self.num_keypoints = 32

        self._log_std_mid = -0.75
        self._log_std_half_span = 1.25

        # 1. Trainable Spatial Visual Head (Levine et al. 2016 Spatial Softmax Keypoints)
        # Learns to project 512 ResNet channels to 32 feature maps and extract 32 continuous (x, y) coordinates
        self.spatial_conv = nn.Conv2d(512, self.num_keypoints, kernel_size=1)
        self.spatial_softmax = SpatialSoftmax2d(height=7, width=7, temperature=1.0)
        self.relu = nn.ReLU()

        # 2. Compact Policy Control Head (64 keypoint coords + 12 onboard telemetry = 76 dims)
        # Drops actor trainable parameters from 453,282 -> 21,474 (-95%) for hyper-fast sample-efficient learning
        self.policy_head = nn.Sequential(
            nn.Linear(self.num_keypoints * 2 + self.num_telemetry, 64),
            nn.ReLU(),
            nn.Linear(64, self.num_actions),
        )

        # Tightly bound initial standard deviation
        self.log_std_parameter = nn.Parameter(torch.zeros(self.num_actions) - 0.5)

        # Non-persistent symmetry parity buffers (move with .to(device) without altering checkpoint state_dict keys)
        spatial_parity = torch.tensor(
            [-1.0, 1.0] * self.num_keypoints, dtype=torch.float32, device=self.device
        )
        telemetry_mirror_signs = torch.tensor(
            TELEMETRY_MIRROR_SIGNS, dtype=torch.float32, device=self.device
        )
        action_mirror_signs = torch.tensor(
            ACTION_MIRROR_SIGNS, dtype=torch.float32, device=self.device
        )
        self.register_buffer("spatial_parity", spatial_parity, persistent=False)
        self.register_buffer(
            "telemetry_mirror_signs", telemetry_mirror_signs, persistent=False
        )
        self.register_buffer(
            "action_mirror_signs", action_mirror_signs, persistent=False
        )

        # Orthogonal / Kaiming Initialization
        nn.init.kaiming_normal_(
            self.spatial_conv.weight, mode="fan_out", nonlinearity="relu"
        )
        nn.init.constant_(self.spatial_conv.bias, 0.0)

        for m in self.policy_head:
            if isinstance(m, nn.Linear):
                nn.init.kaiming_normal_(m.weight, mode="fan_out", nonlinearity="relu")
                if m.bias is not None:
                    nn.init.constant_(m.bias, 0.0)

        # Continuous Control output layer: small gain
        nn.init.orthogonal_(self.policy_head[-1].weight, gain=0.01)
        nn.init.constant_(self.policy_head[-1].bias, 0.0)

    def set_mode(self, mode: str) -> None:
        self.train(mode == "train")

    @property
    def resnet_backbone(self) -> nn.Sequential:
        """Lazily instantiated frozen ResNet-18 backbone (excluded from state_dict & trainable params)."""
        backbone = self.__dict__.get("_resnet_backbone", None)
        target_device = self.spatial_conv.weight.device
        if backbone is None:
            backbone = load_resnet18_backbone(device=target_device, pretrained=True)
            self.__dict__["_resnet_backbone"] = backbone
        elif next(backbone.parameters()).device != target_device:
            backbone = backbone.to(target_device)
            self.__dict__["_resnet_backbone"] = backbone
        return backbone

    def get_log_std(self, role: str = "") -> torch.Tensor:
        """Smooth invertible bijection mapping (-inf, +inf) -> (-2.0, +0.5) with nonzero gradient everywhere."""
        return self._log_std_mid + self._log_std_half_span * torch.tanh(
            self.log_std_parameter / self._log_std_half_span
        )

    def _extract_keypoints_from_image(self, img: torch.Tensor) -> torch.Tensor:
        """Equivariant visual frontend G(I) on 4D image tensor (B, 3, 224, 224)."""
        if img.dim() == 4 and img.shape[1] != 3 and img.shape[-1] == 3:
            img = preprocess_camera_tensor(img, vision_mode=self.vision_mode)
        elif img.dtype == torch.uint8:
            img = preprocess_camera_tensor(img, vision_mode=self.vision_mode)

        backbone = self.resnet_backbone
        if self.equivariant:
            img_pair = torch.cat([img, torch.flip(img, dims=[-1])], dim=0)
            with torch.no_grad():
                feat_pair = backbone(img_pair)
            v_pair = self.relu(self.spatial_conv(feat_pair))
            f_pair = self.spatial_softmax(v_pair)
            f_pos, f_neg = f_pair.chunk(2, dim=0)
            parity = self.spatial_parity.to(device=f_pos.device, dtype=f_pos.dtype)
            return 0.5 * (f_pos + parity * f_neg)
        else:
            with torch.no_grad():
                feat = backbone(img)
            v = self.relu(self.spatial_conv(feat))
            return self.spatial_softmax(v)

    def _extract_keypoints_from_feature_map(
        self, visual_map: torch.Tensor
    ) -> torch.Tensor:
        """Equivariant keypoint extraction from (B, 512, 7, 7) ResNet feature map."""
        if self.equivariant:
            map_pair = torch.cat(
                [visual_map, torch.flip(visual_map, dims=[-1])], dim=0
            )
            v_pair = self.relu(self.spatial_conv(map_pair))
            f_pair = self.spatial_softmax(v_pair)
            f_pos, f_neg = f_pair.chunk(2, dim=0)
            parity = self.spatial_parity.to(device=f_pos.device, dtype=f_pos.dtype)
            return 0.5 * (f_pos + parity * f_neg)
        else:
            v = self.relu(self.spatial_conv(visual_map))
            return self.spatial_softmax(v)

    def G(self, img_or_feat: torch.Tensor) -> torch.Tensor:
        """Equivariant visual frontend G(I) satisfying G(flip(I)) == spatial_parity * G(I)."""
        if img_or_feat.dim() == 4 and (
            img_or_feat.shape[1] == 3 or img_or_feat.shape[-1] == 3
        ):
            return self._extract_keypoints_from_image(img_or_feat)
        if img_or_feat.dim() == 4 and img_or_feat.shape[1] == 512:
            return self._extract_keypoints_from_feature_map(img_or_feat)
        if img_or_feat.dim() == 2 and img_or_feat.shape[1] >= self.num_visual_features:
            visual_map = img_or_feat[:, : self.num_visual_features].view(-1, 512, 7, 7)
            return self._extract_keypoints_from_feature_map(visual_map)
        raise ValueError(
            f"Unsupported input shape {tuple(img_or_feat.shape)} for visual frontend G(I)."
        )

    def extract_visual_features(self, img_or_feat: torch.Tensor) -> torch.Tensor:
        """Alias for G(I)."""
        return self.G(img_or_feat)

    def _mask_actor_telemetry(self, raw_telemetry: torch.Tensor) -> torch.Tensor:
        telemetry = raw_telemetry.clone()
        # Strictly mask privileged coordinates (lat_err=8, head_err=9, distance=11)
        # and autoregressive action feedback (prev_steer=5, prev_drive=6) for Actor.
        # Slot 10 (lookahead_kappa) remains unmasked so Actor receives continuous
        # 5-point windowed lookahead curvature during the 1.5m pre-braking corridor.
        telemetry[:, 5] = 0.0
        telemetry[:, 6] = 0.0
        telemetry[:, 8] = 0.0
        telemetry[:, 9] = 0.0
        telemetry[:, 11] = 0.0
        for idx in ACTOR_MASKED_TELEMETRY_INDICES:
            telemetry[:, idx] = 0.0
        return telemetry

    def _forward_equivariant_head(
        self, k: torch.Tensor, tel_masked: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        parity = self.spatial_parity.to(device=k.device, dtype=k.dtype)
        tel_signs = self.telemetry_mirror_signs.to(
            device=tel_masked.device, dtype=tel_masked.dtype
        )
        act_signs = self.action_mirror_signs.to(device=k.device, dtype=k.dtype)

        feat_pos = torch.cat([k, tel_masked], dim=-1)
        raw_pos = self.policy_head(feat_pos)
        if self.equivariant:
            feat_neg = torch.cat([parity * k, tel_signs * tel_masked], dim=-1)
            raw_neg = self.policy_head(feat_neg)
            pre_act = 0.5 * (raw_pos + act_signs * raw_neg)
        else:
            raw_neg = raw_pos
            pre_act = raw_pos
        return torch.tanh(pre_act), raw_pos, raw_neg

    def raw_policy_logits(
        self,
        obs_or_img: torch.Tensor,
        telemetry: torch.Tensor | None = None,
    ) -> torch.Tensor:
        """Return unsymmetrized raw policy_head(feat_pos) logits for auxiliary mirror regularization."""
        if telemetry is not None:
            if obs_or_img.dim() == 2 and obs_or_img.shape[1] == self.num_keypoints * 2:
                k = obs_or_img
            else:
                k = self.G(obs_or_img)
            tel_masked = self._mask_actor_telemetry(telemetry)
        elif obs_or_img.dim() == 2 and obs_or_img.shape[1] >= self.num_visual_features + self.num_telemetry:
            visual_map = obs_or_img[:, : self.num_visual_features].view(-1, 512, 7, 7)
            k = self._extract_keypoints_from_feature_map(visual_map)
            tel_masked = self._mask_actor_telemetry(
                obs_or_img[
                    :,
                    self.num_visual_features : self.num_visual_features
                    + self.num_telemetry,
                ]
            )
        elif obs_or_img.dim() == 2 and obs_or_img.shape[1] == self.num_keypoints * 2 + self.num_telemetry:
            k = obs_or_img[:, : self.num_keypoints * 2]
            tel_masked = self._mask_actor_telemetry(
                obs_or_img[:, self.num_keypoints * 2 :]
            )
        else:
            raise ValueError(
                f"Cannot extract raw_policy_logits from shape {tuple(obs_or_img.shape)} without telemetry."
            )
        feat_pos = torch.cat([k, tel_masked], dim=-1)
        return self.policy_head(feat_pos)

    def forward_from_image(
        self, img: torch.Tensor, telemetry: torch.Tensor
    ) -> torch.Tensor:
        """Evaluate equivariant actor action mean directly from (B, 3, 224, 224) image and (B, 12) telemetry."""
        k = self._extract_keypoints_from_image(img)
        tel_masked = self._mask_actor_telemetry(telemetry)
        action_mean, _, _ = self._forward_equivariant_head(k, tel_masked)
        return action_mean

    def compute(self, inputs: dict[str, Any], role: str = ""):
        obs = inputs.get("states", None)
        if obs is None:
            obs = inputs.get("observations", None)

        log_std = self.get_log_std(role=role)
        extra_outputs: dict[str, Any] = {"log_std": log_std}

        if "rgb" in inputs and "telemetry" in inputs:
            k = self._extract_keypoints_from_image(inputs["rgb"])
            tel_masked = self._mask_actor_telemetry(inputs["telemetry"])
            action_mean, raw_pos, raw_neg = self._forward_equivariant_head(
                k, tel_masked
            )
            extra_outputs["raw_logits"] = raw_pos
            extra_outputs["raw_logits_mirror"] = raw_neg
        elif obs is not None and obs.dim() == 4:
            k = self._extract_keypoints_from_image(obs)
            tel_raw = inputs.get(
                "telemetry",
                torch.zeros(
                    (obs.shape[0], self.num_telemetry),
                    device=obs.device,
                    dtype=k.dtype,
                ),
            )
            tel_masked = self._mask_actor_telemetry(tel_raw)
            action_mean, raw_pos, raw_neg = self._forward_equivariant_head(
                k, tel_masked
            )
            extra_outputs["raw_logits"] = raw_pos
            extra_outputs["raw_logits_mirror"] = raw_neg
        elif (
            obs is not None
            and obs.dim() == 2
            and obs.shape[1] >= self.num_visual_features + self.num_telemetry
        ):
            visual_flat = obs[:, : self.num_visual_features]
            telemetry = self._mask_actor_telemetry(
                obs[
                    :,
                    self.num_visual_features : self.num_visual_features
                    + self.num_telemetry,
                ]
            )
            # 2D Spatial Keypoint Processing: (B, 512, 7, 7) -> (B, 32, 7, 7) -> (B, 64)
            visual_map = visual_flat.view(-1, 512, 7, 7)
            coords = self._extract_keypoints_from_feature_map(visual_map)
            action_mean, raw_pos, raw_neg = self._forward_equivariant_head(
                coords, telemetry
            )
            extra_outputs["raw_logits"] = raw_pos
            extra_outputs["raw_logits_mirror"] = raw_neg
        elif (
            obs is not None
            and obs.dim() == 2
            and obs.shape[1] == self.num_keypoints * 2 + self.num_telemetry
        ):
            coords = obs[:, : self.num_keypoints * 2]
            telemetry = self._mask_actor_telemetry(obs[:, self.num_keypoints * 2 :])
            action_mean, raw_pos, raw_neg = self._forward_equivariant_head(
                coords, telemetry
            )
            extra_outputs["raw_logits"] = raw_pos
            extra_outputs["raw_logits_mirror"] = raw_neg
        else:
            batch_sz = obs.shape[0] if obs is not None else 1
            action_mean = torch.zeros(
                (batch_sz, self.num_actions), device=self.device
            )

        return action_mean, log_std, extra_outputs

    def act(self, inputs: dict[str, Any], role: str = ""):
        """Unified GaussianMixin.act supporting both skrl 1.x and skrl 2.1.0 with 3-tuple return."""
        mean_actions, log_std, outputs = self.compute(inputs, role)

        self._log_std = log_std
        self._num_samples = mean_actions.shape[0]

        dist = torch.distributions.Normal(mean_actions, log_std.exp())
        self._distribution = dist
        self._g_distribution = dist

        actions = dist.rsample()
        if getattr(self, "_clip_actions", False):
            actions = torch.clamp(
                actions, min=self._clip_actions_min, max=self._clip_actions_max
            )
        elif getattr(self, "_g_clip_actions", False) and getattr(
            self, "_g_min_actions", None
        ) is not None:
            actions = torch.clamp(
                actions, min=self._g_min_actions, max=self._g_max_actions
            )

        taken_actions = inputs.get("taken_actions", actions)
        log_prob = dist.log_prob(taken_actions)
        reduction = getattr(self, "_reduction", getattr(self, "_g_reduction", torch.sum))
        if reduction is not None:
            log_prob = reduction(log_prob, dim=-1)
        if log_prob.dim() != actions.dim():
            log_prob = log_prob.unsqueeze(-1)

        outputs["mean_actions"] = mean_actions
        outputs["log_std"] = log_std
        outputs["log_prob"] = log_prob
        return actions, log_prob, outputs


class ARCProCritic(DeterministicMixin, Model):
    def __init__(
        self, observation_space, action_space, device, clip_actions: bool = False
    ):
        Model.__init__(
            self,
            observation_space=observation_space,
            action_space=action_space,
            device=device,
        )
        DeterministicMixin.__init__(self, clip_actions=clip_actions)

        # Privileged Critic Head (Takes 12 ground-truth telemetry features for stable value estimation)
        self.critic_head = nn.Sequential(
            nn.Linear(TELEMETRY_DIM, 256),
            nn.ReLU(),
            nn.Linear(256, 256),
            nn.ReLU(),
            nn.Linear(256, 1),
        )

        # Running standard scaler buffers for 12D telemetry vector (persisted in checkpoints; legacy dicts may omit them)
        self.register_buffer(
            "obs_running_mean",
            torch.zeros(TELEMETRY_DIM, dtype=torch.float32, device=self.device),
            persistent=True,
        )
        self.register_buffer(
            "obs_running_var",
            torch.ones(TELEMETRY_DIM, dtype=torch.float32, device=self.device),
            persistent=True,
        )
        self.register_buffer(
            "obs_count",
            torch.tensor(1.0, dtype=torch.float32, device=self.device),
            persistent=True,
        )

        for m in self.critic_head:
            if isinstance(m, nn.Linear):
                nn.init.kaiming_normal_(m.weight, mode="fan_out", nonlinearity="relu")
                if m.bias is not None:
                    nn.init.constant_(m.bias, 0.0)

        # Well-conditioned orthogonal initialization for final value projection layer
        nn.init.orthogonal_(self.critic_head[-1].weight, gain=1.0)
        nn.init.constant_(self.critic_head[-1].bias, 0.0)

    def set_mode(self, mode: str) -> None:
        self.train(mode == "train")

    def _load_from_state_dict(
        self,
        state_dict: dict[str, Any],
        prefix: str,
        local_metadata: dict[str, Any],
        strict: bool,
        missing_keys: list[str],
        unexpected_keys: list[str],
        error_msgs: list[str],
    ) -> None:
        """Preserve default running telemetry stats when loading legacy checkpoints without them."""
        legacy_stat_defaults = {
            f"{prefix}obs_running_mean": (self.obs_running_mean, 0.0),
            f"{prefix}obs_running_var": (self.obs_running_var, 1.0),
            f"{prefix}obs_count": (self.obs_count, 1.0),
        }
        missing_stat_keys = [
            key for key in legacy_stat_defaults if key not in state_dict
        ]
        prev_missing_len = len(missing_keys)
        super()._load_from_state_dict(
            state_dict,
            prefix,
            local_metadata,
            strict,
            missing_keys,
            unexpected_keys,
            error_msgs,
        )
        if missing_stat_keys:
            missing_set = set(missing_stat_keys)
            missing_keys[prev_missing_len:] = [
                k for k in missing_keys[prev_missing_len:] if k not in missing_set
            ]
            with torch.no_grad():
                for key in missing_stat_keys:
                    buf, fill_val = legacy_stat_defaults[key]
                    buf.fill_(fill_val)

    def _extract_raw_telemetry(self, state: Any) -> torch.Tensor:
        if isinstance(state, dict):
            if "critic" in state:
                vec = state["critic"]
            elif "policy" in state:
                vec = state["policy"]
            else:
                vec = next(iter(state.values()))
        else:
            if state.shape[1] > TELEMETRY_DIM:
                vec = state[:, -TELEMETRY_DIM:]
            else:
                vec = state
        return vec

    @staticmethod
    def transform_telemetry(vec: torch.Tensor) -> torch.Tensor:
        """Bounded, smooth C^inf compression of unbounded odometer slot 11 (distance)."""
        out = vec.clone()
        if out.shape[-1] >= 12:
            # Smooth signed log compression: asinh(d / 10.0) maps [0, 500m] -> [0, 4.605]
            # with strictly positive derivative 1 / (10 * sqrt(1 + (d/10)^2)) > 0 everywhere.
            out[:, 11] = torch.asinh(out[:, 11] / 10.0)
        return out

    @torch.no_grad()
    def update_telemetry_stats(self, state: torch.Tensor) -> None:
        """Update running mean and variance of the transformed 12D Critic telemetry."""
        vec = self.transform_telemetry(self._extract_raw_telemetry(state)).to(
            device=self.obs_running_mean.device, dtype=torch.float32
        )
        if vec.shape[0] < 2:
            return
        batch_mean = vec.mean(dim=0)
        batch_var = vec.var(dim=0, unbiased=False)
        batch_count = float(vec.shape[0])

        delta = batch_mean - self.obs_running_mean
        total_count = self.obs_count + batch_count
        m2 = (
            self.obs_running_var * self.obs_count
            + batch_var * batch_count
            + (delta**2) * self.obs_count * batch_count / total_count
        )
        self.obs_running_mean.copy_(
            self.obs_running_mean + delta * (batch_count / total_count)
        )
        self.obs_running_var.copy_(torch.clamp(m2 / total_count, min=1e-4))
        self.obs_count.copy_(total_count)

    def compute(self, inputs: dict[str, Any], role: str = ""):
        state = inputs.get("states", None)
        if state is None:
            state = inputs.get("observations", None)
        vec = self.transform_telemetry(self._extract_raw_telemetry(state))
        mean = self.obs_running_mean.to(device=vec.device, dtype=vec.dtype)
        var = self.obs_running_var.to(device=vec.device, dtype=vec.dtype)
        vec_norm = (vec - mean) / (torch.sqrt(var) + 1e-8)
        return self.critic_head(vec_norm), {}

    def act(self, inputs: dict[str, Any], role: str = ""):
        """Unified DeterministicMixin.act returning 3-tuple (values, None, outputs)."""
        actions, outputs = self.compute(inputs, role)
        if getattr(self, "_clip_actions", False):
            actions = torch.clamp(
                actions, min=self._clip_actions_min, max=self._clip_actions_max
            )
        elif getattr(self, "_d_clip_actions", False) and getattr(
            self, "_d_min_actions", None
        ) is not None:
            actions = torch.clamp(
                actions, min=self._d_min_actions, max=self._d_max_actions
            )
        return actions, None, outputs
