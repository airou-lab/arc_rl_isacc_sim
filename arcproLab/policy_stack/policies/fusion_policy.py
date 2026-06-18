"""Fusion Features Extractor for Multi-Modal Perception (ResNet-18 backbone).

Architecture:
    Visual Stream:  ResNet-18 (standard ImageNet stem) -> AdaptiveAvgPool -> 512
                    -> Linear(512, 256) + ReLU         -> 256-dim visual features
    Physics Stream: Identity passthrough               -> vec_dim (12 by default)
    Fusion:         Concatenate + LayerNorm            -> 268-dim output

Port of `policies/fusion_policy.py` from arc_rl_isacc_policy
(commit e200b27 "feat(policy): swap NatureCNN for ResNet-18 at 224x224"),
adapted for the sim's observation schema: keys are `tiled_camera` and
`telemetry` (the sim's PolicyCfg term names) rather than `image` / `vec`.
Override via constructor kwargs if a wrapper renames them.

Resolution: 224 x 224
    Matches the canonical ResNet-18 ImageNet input. The pretrained filter
    scales, BN running stats, and effective receptive fields transfer as
    designed at this shape.

    Spatial trace at 224x224 with the standard stem:
        Input:     (B, 3, 224, 224)
        conv1 7x7 s=2 -> (B, 64, 112, 112)
        maxpool s=2   -> (B, 64, 56, 56)
        layer1        -> (B, 64, 56, 56)
        layer2 s=2    -> (B, 128, 28, 28)
        layer3 s=2    -> (B, 256, 14, 14)
        layer4 s=2    -> (B, 512, 7, 7)
        avgpool       -> (B, 512, 1, 1) -> flatten -> 512

RL-specific detail (BatchNorm):
    BN is brittle in RL: rollouts run at effective batch size 1 while
    training runs at larger batches, and running stats drift. We pin
    every BN module to eval mode and re-pin on every .train() call.
    Affine params (gamma, beta) still receive gradients; only running
    stats are frozen. Standard pattern for pretrained CNN backbones in RL.

PVP note:
    ResNet operates on the image only. The 12-element telemetry vector
    flows through as identity passthrough. PVP-zeroed slots remain zeroed
    by the env upstream. No new privileged signal introduced.

Observation Space Contract (as wired in arcpro_env_cfg.py PolicyCfg):
    gymnasium.spaces.Dict with
        'tiled_camera': Box(224, 224, 3) uint8 - RGB camera image
        'telemetry'   : Box(N,)         float32 - telemetry vector (N=12)

    SB3 transposes (H, W, C) -> (C, H, W) and scales uint8 -> float32 [0, 1]
    before forward() is called. When pretrained=True we additionally apply
    ImageNet mean/std normalization inside this module.
"""
from __future__ import annotations

import warnings
from typing import Optional

import torch
import torch.nn as nn
from gymnasium import spaces
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor

_IMAGENET_MEAN = (0.485, 0.456, 0.406)
_IMAGENET_STD = (0.229, 0.224, 0.225)

# Visual feature dim after projection. Kept at 256 so fused dim stays
# 256 + 12 = 268 and downstream code (LSTM input, tests) does not move.
_VISUAL_FEATURES_DIM = 256


def _build_resnet18(pretrained: bool, cifar_stem: bool) -> nn.Module:
    """Build a torchvision ResNet-18 with the final FC removed.

    Default keeps the canonical 7x7 s=2 conv1 + maxpool stem (ImageNet).
    `cifar_stem=True` swaps the stem for the small-input variant
    (3x3 s=1, no early maxpool); preserved as an ablation hook.
    """
    from torchvision.models import resnet18, ResNet18_Weights

    if pretrained:
        try:
            weights = ResNet18_Weights.IMAGENET1K_V1
            backbone = resnet18(weights=weights)
        except Exception as exc:
            warnings.warn(
                f"Could not load pretrained ResNet-18 weights ({exc!r}). "
                "Falling back to random init. This is fine for offline tests "
                "but degrades sim-to-real performance for real training runs.",
                RuntimeWarning,
                stacklevel=2,
            )
            backbone = resnet18(weights=None)
    else:
        backbone = resnet18(weights=None)

    if cifar_stem:
        new_conv1 = nn.Conv2d(3, 64, kernel_size=3, stride=1, padding=1, bias=False)
        nn.init.kaiming_normal_(new_conv1.weight, mode="fan_out", nonlinearity="relu")
        backbone.conv1 = new_conv1
        backbone.maxpool = nn.Identity()

    backbone.fc = nn.Identity()
    return backbone


class FusionFeaturesExtractor(BaseFeaturesExtractor):
    """Dual-stream fusion network with a ResNet-18 visual backbone.

    Input observation (after SB3 preprocessing):
        image_key: (B, 3, H, W) float32 in [0, 1]
        vec_key  : (B, vec_dim) float32

    Output:
        (B, 256 + vec_dim) LayerNorm'd fused features.
        For the standard 12-dim telemetry vector this is (B, 268).
    """

    def __init__(
        self,
        observation_space: spaces.Dict,
        features_dim: int = 268,
        backbone: str = "resnet18",
        pretrained: bool = True,
        cifar_stem: bool = False,
        freeze_backbone: bool = False,
        apply_imagenet_normalization: Optional[bool] = None,
        image_key: str = "tiled_camera",
        vec_key: str = "telemetry",
    ):
        """
        Args:
            observation_space: Dict space with image_key (H, W, 3) uint8 and
                vec_key (N,) float32.
            features_dim: Ignored. Output dim is recomputed as 256 + vec_dim.
                Retained for SB3 features_extractor_kwargs compatibility.
            backbone: Visual backbone name. Only "resnet18" is implemented.
            pretrained: Load ImageNet-pretrained weights. Default True.
            cifar_stem: Use the small-input 3x3 s=1 stem with no early maxpool.
                Default False; reserved as an ablation hook.
            freeze_backbone: Freeze ResNet params so only the projection and
                fusion LayerNorm train. Useful for cheap eval runs.
            apply_imagenet_normalization: If None (default), follows
                `pretrained`. Set explicitly to override for ablations.
            image_key: Dict key for the image obs term. Defaults to the sim's
                "tiled_camera"; pass "image" if a wrapper renamed it.
            vec_key: Dict key for the vec obs term. Defaults to the sim's
                "telemetry"; pass "vec" if a wrapper renamed it.
        """
        if backbone != "resnet18":
            raise ValueError(
                f"Unsupported visual backbone '{backbone}'. "
                "Only 'resnet18' is implemented; add a builder for new variants."
            )
        if image_key not in observation_space.spaces:
            raise KeyError(
                f"image_key='{image_key}' not in observation_space. "
                f"Available: {list(observation_space.spaces)}"
            )
        if vec_key not in observation_space.spaces:
            raise KeyError(
                f"vec_key='{vec_key}' not in observation_space. "
                f"Available: {list(observation_space.spaces)}"
            )

        vec_dim = observation_space[vec_key].shape[0]
        total_dim = _VISUAL_FEATURES_DIM + vec_dim
        super().__init__(observation_space, features_dim=total_dim)

        self._image_key = image_key
        self._vec_key = vec_key

        if apply_imagenet_normalization is None:
            apply_imagenet_normalization = pretrained
        self._apply_imagenet_norm = apply_imagenet_normalization

        mean = torch.tensor(_IMAGENET_MEAN, dtype=torch.float32).view(1, 3, 1, 1)
        std = torch.tensor(_IMAGENET_STD, dtype=torch.float32).view(1, 3, 1, 1)
        self.register_buffer("imagenet_mean", mean)
        self.register_buffer("imagenet_std", std)

        self.backbone = _build_resnet18(pretrained=pretrained, cifar_stem=cifar_stem)
        if freeze_backbone:
            for p in self.backbone.parameters():
                p.requires_grad = False

        self.visual_proj = nn.Sequential(
            nn.Linear(512, _VISUAL_FEATURES_DIM),
            nn.ReLU(),
        )

        self.fusion_norm = nn.LayerNorm(total_dim)

        self._set_bn_eval()

    def _set_bn_eval(self) -> None:
        for m in self.backbone.modules():
            if isinstance(m, (nn.BatchNorm1d, nn.BatchNorm2d, nn.BatchNorm3d)):
                m.eval()

    def train(self, mode: bool = True):
        """Override so SB3's per-update self.policy.train(True) does not
        re-enable BN running-stat updates. BN affine params still train.
        """
        super().train(mode)
        self._set_bn_eval()
        return self

    def forward(self, observation: dict) -> torch.Tensor:
        x = observation[self._image_key]
        if self._apply_imagenet_norm:
            x = (x - self.imagenet_mean) / self.imagenet_std

        visual_feats = self.backbone(x)               # (B, 512)
        visual_feats = self.visual_proj(visual_feats)  # (B, 256)

        physics_feats = observation[self._vec_key]    # (B, vec_dim)
        fused = torch.cat([visual_feats, physics_feats], dim=1)
        return self.fusion_norm(fused)
