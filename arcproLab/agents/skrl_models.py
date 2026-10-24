import torch
import torch.nn as nn
from skrl.models.torch import DeterministicMixin, GaussianMixin, Model


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
        flat_x = x.view(B, C, H * W) / self.temperature
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
        clip_actions=True,
        clip_log_std=True,
        min_log_std=-20,
        max_log_std=2,
        reduction="sum",
    ):
        Model.__init__(self, observation_space, action_space, device)
        GaussianMixin.__init__(
            self, clip_actions, clip_log_std, min_log_std, max_log_std, reduction
        )

        self.num_visual_features = 25088  # 512 channels x 7 x 7 spatial grid
        self.num_telemetry = 12
        self.num_keypoints = 32

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

    def compute(self, inputs, role=""):
        obs = inputs["states"]
        if obs.shape[1] >= self.num_visual_features + self.num_telemetry:
            visual_flat = obs[:, : self.num_visual_features]
            telemetry = obs[
                :,
                self.num_visual_features : self.num_visual_features
                + self.num_telemetry,
            ].clone()
            # Strictly mask privileged coordinates (lat_err=8, head_err=9, distance=11) for Actor.
            # Slot 10 (lookahead_kappa) remains unmasked for pre-braking.
            telemetry[:, 8] = 0.0
            telemetry[:, 9] = 0.0
            telemetry[:, 11] = 0.0

            # 2D Spatial Keypoint Processing: (B, 512, 7, 7) -> (B, 32, 7, 7) -> (B, 64)
            visual_map = visual_flat.view(-1, 512, 7, 7)
            v = self.relu(self.spatial_conv(visual_map))
            coords = self.spatial_softmax(v)

            combined = torch.cat([coords, telemetry], dim=1)
            action_mean = torch.tanh(self.policy_head(combined))
        else:
            # Fallback for telemetry-only states
            action_mean = torch.zeros(
                (obs.shape[0], self.num_actions), device=self.device
            )

        return action_mean, self.log_std_parameter, {}


class ARCProCritic(DeterministicMixin, Model):
    def __init__(self, observation_space, action_space, device, clip_actions=False):
        Model.__init__(self, observation_space, action_space, device)
        DeterministicMixin.__init__(self, clip_actions)

        # Privileged Critic Head (Takes 12 ground-truth telemetry features for stable value estimation)
        self.critic_head = nn.Sequential(
            nn.Linear(12, 256),
            nn.ReLU(),
            nn.Linear(256, 256),
            nn.ReLU(),
            nn.Linear(256, 1),
        )

        for m in self.critic_head:
            if isinstance(m, nn.Linear):
                nn.init.kaiming_normal_(m.weight, mode="fan_out", nonlinearity="relu")
                if m.bias is not None:
                    nn.init.constant_(m.bias, 0)

    def compute(self, inputs, role=""):
        state = inputs["states"]
        if isinstance(state, dict):
            if "critic" in state:
                vec = state["critic"]
            elif "policy" in state:
                vec = state["policy"]
            else:
                vec = next(iter(state.values()))
        else:
            if state.shape[1] > 12:
                vec = state[:, -12:]
            else:
                vec = state

        return self.critic_head(vec), {}
