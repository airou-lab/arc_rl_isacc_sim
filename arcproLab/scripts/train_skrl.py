import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(
    description="Train SKRL AAC policy for ARCPro Lane Following."
)
parser.add_argument(
    "--num_envs",
    type=int,
    default=16,
    help="Number of parallel simulation environments.",
)
parser.add_argument("--seed", type=int, default=42, help="Seed for the environment.")
parser.add_argument(
    "--total_timesteps", type=int, default=5000000, help="Total timesteps to train."
)
parser.add_argument(
    "--resume", type=str, default="", help="Path to checkpoint to resume from."
)
parser.add_argument(
    "--initial_step",
    type=int,
    default=None,
    help="Initial step offset for cumulative step tracking.",
)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import os
import re
import subprocess
import sys
import warnings
from datetime import datetime

import torch

warnings.filterwarnings("ignore", category=DeprecationWarning)

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))
ARCPRO_LAB_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)
if ARCPRO_LAB_DIR not in sys.path:
    sys.path.insert(0, ARCPRO_LAB_DIR)

import gymnasium as gym

# Import the new SKRL models
from agents.skrl_models import ARCProActor, ARCProCritic
from agents.skrl_wrappers import SKRLFlattenWrapper
from arcpro_env_cfg import ARCProEnvCfg
from isaaclab.envs import ManagerBasedRLEnv
from skrl.agents.torch.ppo import PPO, PPO_DEFAULT_CONFIG
from skrl.envs.wrappers.torch import IsaacLabWrapper
from skrl.memories.torch import RandomMemory
from skrl.trainers.torch import SequentialTrainer


class TelemetryPPO(PPO):
    def __init__(self, *args, initial_step: int = 0, num_envs: int = 16, **kwargs):
        super().__init__(*args, **kwargs)
        self.initial_step = initial_step
        self.num_envs = num_envs
        self._print_counter = 0
        self._last_ep_len = 0.0
        self._last_ep_len_max = 0.0
        self._last_ep_rew = 0.0
        self._last_ep_rew_max = 0.0

        # Differential Learning Rates:
        # 1. Visual representation head (spatial_conv, spatial_fc, layer_norm): 3e-5 (protects visual stability)
        # 2. Policy control head (policy_head, log_std): 3e-4 (10x boost for rapid steering adaptation)
        # 3. Value critic head (critic_head): 1e-4 (fast value baseline tracking)
        if self.policy is not None and self.value is not None:
            visual_params = (
                list(self.policy.spatial_conv.parameters())
                + list(self.policy.spatial_fc.parameters())
                + list(self.policy.layer_norm.parameters())
            )
            visual_ids = set(id(p) for p in visual_params)
            control_params = [
                p for p in self.policy.parameters() if id(p) not in visual_ids
            ]

            self.optimizer = torch.optim.Adam(
                [
                    {"params": visual_params, "lr": 3e-5},
                    {"params": control_params, "lr": 3e-4},
                    {"params": list(self.value.parameters()), "lr": 1e-4},
                ]
            )
            self.checkpoint_modules["optimizer"] = self.optimizer

    def load(self, path: str) -> None:
        """Robust checkpoint loading that handles parameter group transitions."""
        modules = torch.load(path, map_location=self.device, weights_only=False)
        if isinstance(modules, dict):
            for name, data in modules.items():
                module = self.checkpoint_modules.get(name, None)
                if module is not None:
                    if name == "optimizer":
                        try:
                            module.load_state_dict(data)
                        except Exception as e:
                            print(
                                f"[TelemetryPPO] Optimizer reinitialized with differential parameter groups ({e})"
                            )
                    elif hasattr(module, "load_state_dict"):
                        module.load_state_dict(data, strict=False)
                        if hasattr(module, "eval"):
                            module.eval()
                else:
                    print(f"[TelemetryPPO] Skipping unknown module: {name}")

    def write_tracking_data(self, timestep: int, timesteps: int) -> None:
        cum_timestep = (timestep + self.initial_step) if timestep is not None else None
        cum_timesteps = (
            (timesteps + self.initial_step) if timesteps is not None else None
        )
        super().write_tracking_data(cum_timestep, cum_timesteps)

    def write_checkpoint(self, timestep: int, timesteps: int) -> None:
        cum_timestep = (timestep + self.initial_step) if timestep is not None else None
        cum_timesteps = (
            (timesteps + self.initial_step) if timesteps is not None else None
        )
        super().write_checkpoint(cum_timestep, cum_timesteps)

    def record_transition(
        self,
        states,
        actions,
        rewards,
        next_states,
        terminated,
        truncated,
        infos,
        timestep,
        timesteps,
    ):
        cum_timestep = (timestep + self.initial_step) if timestep is not None else None
        cum_timesteps = (
            (timesteps + self.initial_step) if timesteps is not None else None
        )
        super().record_transition(
            states,
            actions,
            rewards,
            next_states,
            terminated,
            truncated,
            infos,
            cum_timestep,
            cum_timesteps,
        )

        speed = 0.0
        track_pct = 0.0
        wp_delta = 0
        if "speed_mps" in infos:
            speed = infos["speed_mps"].mean().item()
            self.track_data("Telemetry / Speed_MPS", speed)
        if "dist_white" in infos:
            self.track_data(
                "Telemetry / Dist_White_m", infos["dist_white"].mean().item()
            )
        if "dist_yellow" in infos:
            self.track_data(
                "Telemetry / Dist_Yellow_m", infos["dist_yellow"].mean().item()
            )
        if "track_progress_pct" in infos:
            track_pct = infos["track_progress_pct"].mean().item()
            self.track_data("Telemetry / Track_Progress_Pct", track_pct)
        if "track_wp_delta" in infos:
            # Cumulative WPs from spawn this episode (obs-layer variable)
            wp_delta = infos["track_wp_delta"].float().mean().item()
            self.track_data("Telemetry / WP_Delta_Cumulative", wp_delta)
        # FIX B: Log the ACTUAL per-step reward delta (rewards.py variable)
        wp_step = 0.0
        if "reward_wp_delta_step" in infos:
            wp_step = infos["reward_wp_delta_step"].float().mean().item()
            self.track_data("Telemetry / WP_Delta_Step_Reward", wp_step)

        # Add custom clean text logging
        self._print_counter += states.shape[0] if states is not None else 1
        if self._print_counter >= 10000:
            ep_len_list = self.tracking_data.get("Episode / Total timesteps (mean)", [])
            ep_len_max_list = self.tracking_data.get(
                "Episode / Total timesteps (max)", []
            )
            ep_rew_list = self.tracking_data.get("Reward / Total reward (mean)", [])
            ep_rew_max_list = self.tracking_data.get("Reward / Total reward (max)", [])

            if len(ep_len_list) > 0 and ep_len_list[-1] != 0:
                self._last_ep_len = ep_len_list[-1]
            if len(ep_len_max_list) > 0 and ep_len_max_list[-1] != 0:
                self._last_ep_len_max = ep_len_max_list[-1]
            if len(ep_rew_list) > 0 and ep_rew_list[-1] != 0:
                self._last_ep_rew = ep_rew_list[-1]
            if len(ep_rew_max_list) > 0 and ep_rew_max_list[-1] != 0:
                self._last_ep_rew_max = ep_rew_max_list[-1]

            cum_step = timestep + self.initial_step
            total_env_samples = cum_step * self.num_envs
            print(
                f"Step {cum_step:7d} ({total_env_samples / 1e6:5.1f}M env) | Rew: {self._last_ep_rew:6.1f} (max {self._last_ep_rew_max:6.1f}) | "
                f"Len: {self._last_ep_len:3.0f} (max {self._last_ep_len_max:3.0f}) | "
                f"Spd: {speed:.2f} | WPs_cum: {wp_delta:.0f} | WPΔ_rew: {wp_step:.2f}"
            )
            sys.stdout.flush()
            self._print_counter = 0


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


def main():
    # 1. Setup Environment Configuration
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    if args_cli.enable_cameras:
        env_cfg.enable_cameras = True
    env_cfg.__post_init__()

    # 2. Create Environment
    env = ManagerBasedRLEnv(cfg=env_cfg)
    env = WarmupActionWrapper(env, warmup_steps=10)

    # 3. Wrap for SKRL
    # The IsaacLabWrapper automatically extracts the "critic" observation group
    # (if it exists) into env.state_space and env.state(), enabling AAC.
    env = IsaacLabWrapper(env)

    # 3.5 Flatten for SKRL RandomMemory limitation
    env = SKRLFlattenWrapper(env)

    # 4. Define Memory (1024 rollouts with frozen ResNet-18)
    memory = RandomMemory(memory_size=1024, num_envs=env.num_envs, device="cuda:0")

    # 5. Define Models
    models = {}
    models["policy"] = ARCProActor(env.observation_space, env.action_space, "cuda:0")

    # Use env.state_space for the Critic if available, fallback to observation_space
    state_space = getattr(env, "state_space", env.observation_space)
    models["value"] = ARCProCritic(state_space, env.action_space, "cuda:0")

    # 6. Configure Agent
    cfg_ppo = PPO_DEFAULT_CONFIG.copy()
    cfg_ppo["rollouts"] = 1024
    cfg_ppo["mini_batches"] = (
        8  # Mini-batch size = (1024 * 64 / 8) = 8,192 transitions per backprop
    )
    cfg_ppo["learning_rate"] = (
        3e-5  # Stable learning rate for fine-tuning 700-step baseline
    )
    cfg_ppo["random_timesteps"] = 0
    cfg_ppo["learning_starts"] = 0
    cfg_ppo["state_preprocessor"] = None
    # Use native PPO KL early stopping and Entropy to prevent policy collapse
    cfg_ppo["kl_threshold"] = (
        0.016  # Relaxed from 0.008 — too tight was causing snap-back collapses
    )
    cfg_ppo["entropy_loss_scale"] = (
        0.010  # Raised to 0.010 for active corner exploration out of straightaway local minima
    )
    cfg_ppo["learning_rate_scheduler"] = None

    # Branch-aware experiment directory to isolate checkpoints by branch
    try:
        branch_name = (
            subprocess.check_output(
                ["git", "rev-parse", "--abbrev-ref", "HEAD"], stderr=subprocess.DEVNULL
            )
            .decode()
            .strip()
        )
        branch_slug = re.sub(r"[^a-zA-Z0-9_\-]", "_", branch_name)
    except Exception:
        branch_slug = "main"

    log_folder = "ppo_skrl" if branch_slug == "main" else f"ppo_skrl_{branch_slug}"
    log_dir = os.path.join("logs", log_folder, datetime.now().strftime("%Y%m%d-%H%M%S"))
    cfg_ppo["experiment"]["directory"] = log_dir
    cfg_ppo["experiment"]["write_interval"] = 100
    cfg_ppo["experiment"]["checkpoint_interval"] = (
        50000  # Save every 50k timesteps (~2.4h) for safe disaster recovery
    )

    # Auto-detect initial_step from resume checkpoint if not explicitly provided
    initial_step = args_cli.initial_step
    if initial_step is None and args_cli.resume:
        match = re.search(r"agent_(\d+)", args_cli.resume)
        if match:
            initial_step = int(match.group(1))
            print(
                f"[Train] Auto-detected initial step offset from checkpoint: {initial_step}"
            )
        else:
            initial_step = 0
    elif initial_step is None:
        initial_step = 0

    agent = TelemetryPPO(
        models=models,
        memory=memory,
        cfg=cfg_ppo,
        observation_space=env.observation_space,
        action_space=env.action_space,
        device="cuda:0",
        initial_step=initial_step,
        num_envs=args_cli.num_envs,
    )

    if args_cli.resume:
        print(f"Resuming from checkpoint: {args_cli.resume}")
        agent.load(args_cli.resume)

    # 7. Train
    print(f"Starting SKRL AAC training for {args_cli.total_timesteps} steps...")
    trainer_cfg = {"timesteps": args_cli.total_timesteps, "disable_progressbar": True}
    trainer = SequentialTrainer(cfg=trainer_cfg, env=env, agents=agent)
    trainer.train()

    env.close()


if __name__ == "__main__":
    main()
