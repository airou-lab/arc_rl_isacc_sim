import torch
import torch.nn as nn
from skrl.models.torch import DeterministicMixin, GaussianMixin, Model


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

        # 1. Trainable Spatial Visual Head (Preserves Left vs Right Line Geometry)
        # Learns to detect yellow line on left / white line on right via PPO backprop
        self.spatial_conv = nn.Conv2d(512, 32, kernel_size=1)
        self.spatial_fc = nn.Linear(32 * 7 * 7, 256)
        self.layer_norm = nn.LayerNorm(256)
        self.relu = nn.ReLU()

        # 2. Policy Control Head (256 vision features + 12 onboard telemetry = 268)
        self.policy_head = nn.Sequential(
            nn.Linear(256 + self.num_telemetry, 128),
            nn.ReLU(),
            nn.Linear(128, self.num_actions),
        )

        # Tightly bound initial standard deviation
        self.log_std_parameter = nn.Parameter(torch.zeros(self.num_actions) - 0.5)

        # Orthogonal Initialization
        nn.init.kaiming_normal_(
            self.spatial_conv.weight, mode="fan_out", nonlinearity="relu"
        )
        nn.init.constant_(self.spatial_conv.bias, 0.0)
        nn.init.kaiming_normal_(
            self.spatial_fc.weight, mode="fan_out", nonlinearity="relu"
        )
        nn.init.constant_(self.spatial_fc.bias, 0.0)

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
            # Disarm auto-centering spring: mask last_steer (slot 5) to 0.0 so network cannot counter-steer
            telemetry[:, 5] = 0.0

            # 2D Spatial Processing: (B, 512, 7, 7) -> (B, 32, 7, 7) -> (B, 1568) -> (B, 256)
            visual_map = visual_flat.view(-1, 512, 7, 7)
            v = self.relu(self.spatial_conv(visual_map))
            v = v.flatten(start_dim=1)
            v = self.relu(self.layer_norm(self.spatial_fc(v)))

            combined = torch.cat([v, telemetry], dim=1)
            raw_action_mean = torch.tanh(self.policy_head(combined))

            # Kinematic curvature feedforward steering prior (Ackermann L/delta_max = 0.33 / 0.68 = 0.485)
            # Slot 10 is lookahead_kappa. On straightaways (kappa ~ 0), steer_bias = 0.0 (preserves straight tracking).
            # In corners (kappa > 0 for left, < 0 for right), provides intentional turn-in bias so vehicle explores
            # and reinforces visual cornering rather than driving straight into the wall.
            kappa = telemetry[:, 10:11]
            steer_bias = torch.clamp(0.485 * kappa, min=-1.0, max=1.0)

            action_steer = torch.clamp(
                raw_action_mean[:, 0:1] + steer_bias, min=-1.0, max=1.0
            )
            # Kinematic curvature feedforward deceleration prior
            # Slot 10 is lookahead_kappa. On straightaways (|kappa| ~ 0), brake_bias = 0.0 (full acceleration available).
            # Approaching sharp corners (|kappa| >= 0.70 to 2.15), brake_bias scales proportionally up to 0.70,
            # reducing drive action from cruising (+0.25) down to negative throttle (-0.45),
            # which commands wheel joint velocity ~11 rad/s (v ~ 0.35 m/s), matching Corner 1 controllable envelope.
            brake_bias = torch.clamp(torch.abs(kappa) * 0.35, min=0.0, max=0.70)
            action_drive = torch.clamp(
                raw_action_mean[:, 1:2] - brake_bias, min=-1.0, max=1.0
            )

            action_mean = torch.cat([action_steer, action_drive], dim=1)
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
