import torch
import torch.nn as nn
from skrl.models.torch import Model, GaussianMixin, DeterministicMixin
from torchvision.models import resnet18, ResNet18_Weights

class ARCProActor(GaussianMixin, Model):
    def __init__(self, observation_space, action_space, device, clip_actions=False,
                 clip_log_std=True, min_log_std=-20, max_log_std=2, reduction="sum"):
        Model.__init__(self, observation_space, action_space, device)
        GaussianMixin.__init__(self, clip_actions, clip_log_std, min_log_std, max_log_std, reduction)
        
        # 1. Vision Backbone (Frozen)
        # We freeze the entire backbone to eliminate PPO KL spikes.
        self.resnet = resnet18(weights=ResNet18_Weights.IMAGENET1K_V1)
        for param in self.resnet.parameters():
            param.requires_grad = False
        self.resnet.fc = nn.Identity()
        
        self.register_buffer("mean", torch.tensor([0.485, 0.456, 0.406]).view(1, 3, 1, 1))
        self.register_buffer("std", torch.tensor([0.229, 0.224, 0.225]).view(1, 3, 1, 1))
        
        # 2. Actor Head (Vision 512 + Telemetry 12)
        self.actor_head = nn.Sequential(
            nn.Linear(512 + 12, 256),
            nn.ReLU(),
            nn.LayerNorm(256),
            nn.Linear(256, 128),
            nn.ReLU(),
            nn.Linear(128, self.num_actions)
        )
        # Tightly bound initial standard deviation to prevent immediate crashes
        self.log_std_parameter = nn.Parameter(torch.zeros(self.num_actions) - 0.5)

        for m in self.actor_head:
            if isinstance(m, nn.Linear):
                nn.init.kaiming_normal_(m.weight, mode='fan_out', nonlinearity='relu')
                if m.bias is not None:
                    nn.init.constant_(m.bias, 0)

    def compute(self, inputs, role=""):
        obs = inputs["states"]
        
        # Unflatten the 1D tensor from SKRLFlattenWrapper
        if obs.dim() == 2 and obs.shape[1] == 150540:
            vec = obs[:, :12]
            img = obs[:, 12:].view(-1, 224, 224, 3)
            use_vision = True
        elif obs.dim() == 2 and obs.shape[1] == 12:
            vec = obs
            use_vision = False
        else:
            raise ValueError(f"ARCProActor expects flattened obs of shape (N, 150540) or (N, 12), got {obs.shape}")
            
        if use_vision:
            img = img.float()
            
            # IsaacLab tiled_camera is usually (B, H, W, C) -> (B, C, H, W)
            if img.dim() == 4 and img.shape[-1] == 3:
                img = img.permute(0, 3, 1, 2)
                
            # FIX: ImageNet-trained layers REQUIRE (img - mean) / std. 
            # The environment passes img in [0, 1]. We MUST apply the ImageNet stats, 
            # otherwise the frozen layers will output garbage.
            if img.max() > 1.0:
                img = img / 255.0
                
            img = (img - self.mean) / self.std
            
            visual_feats = self.resnet(img)
        else:
            visual_feats = torch.zeros((vec.shape[0], 512), device=vec.device)
            
        # Concatenate Vision and Telemetry
        combined = torch.cat([visual_feats, vec], dim=1)
        
        return self.actor_head(combined), self.log_std_parameter, {}


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
            else:
                vec = state
            
        return self.critic_head(vec), {}
