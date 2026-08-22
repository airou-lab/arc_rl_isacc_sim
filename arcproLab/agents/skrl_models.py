import torch
import torch.nn as nn
from skrl.models.torch import Model, GaussianMixin, DeterministicMixin
from torchvision.models import resnet18, ResNet18_Weights

class ARCProActor(GaussianMixin, Model):
    def __init__(self, observation_space, action_space, device, clip_actions=True,
                 clip_log_std=True, min_log_std=-20, max_log_std=2, reduction="sum"):
        Model.__init__(self, observation_space, action_space, device)
        GaussianMixin.__init__(self, clip_actions, clip_log_std, min_log_std, max_log_std, reduction)
        
        # Actor Head (Vision 512 + Telemetry 12)
        # The input is already pre-computed by the Environment Wrapper using ResNet!
        self.actor_head = nn.Sequential(
            nn.Linear(self.num_observations, 256),
            nn.ReLU(),
            nn.LayerNorm(256),
            nn.Linear(256, 128),
            nn.ReLU(),
            nn.Linear(128, self.num_actions)
        )
        
        # Tightly bound initial standard deviation
        self.log_std_parameter = nn.Parameter(torch.zeros(self.num_actions) - 0.5)

        for m in self.actor_head:
            if isinstance(m, nn.Linear):
                nn.init.kaiming_normal_(m.weight, mode='fan_out', nonlinearity='relu')
                if m.bias is not None:
                    nn.init.constant_(m.bias, 0)

    def compute(self, inputs, role=""):
        # The SKRLFlattenWrapper has already run the frozen ResNet
        # so `inputs["states"]` is perfectly sized at (N, 524)
        obs = inputs["states"]
        return torch.tanh(self.actor_head(obs)), self.log_std_parameter, {}


class ARCProCritic(DeterministicMixin, Model):
    def __init__(self, observation_space, action_space, device, clip_actions=False):
        Model.__init__(self, observation_space, action_space, device)
        DeterministicMixin.__init__(self, clip_actions)
        
        # 1. Privileged Critic Head (MLP only, no vision)
        self.critic_head = nn.Sequential(
            nn.Linear(12, 256),
            nn.ReLU(),
            nn.Linear(256, 256),
            nn.ReLU(),
            nn.Linear(256, 1)
        )
        
        for m in self.critic_head:
            if isinstance(m, nn.Linear):
                nn.init.kaiming_normal_(m.weight, mode='fan_out', nonlinearity='relu')
                if m.bias is not None:
                    nn.init.constant_(m.bias, 0)

    def compute(self, inputs, role=""):
        # The SKRL agent will pass the unmasked privileged state here
        state = inputs["states"]
        
        if isinstance(state, dict):
            if "critic" in state:
                vec = state["critic"]
            elif "policy" in state:
                vec = state["policy"]
            else:
                vec = next(iter(state.values()))
        else:
            if state.dim() == 2 and state.shape[1] == 150540:
                vec = state[:, :12]
            elif state.dim() == 2 and state.shape[1] == 524:
                # SKRLFlattenWrapper returns [visual_feats (512), telemetry (12)]
                vec = state[:, -12:]
            else:
                vec = state
            
        return self.critic_head(vec), {}
