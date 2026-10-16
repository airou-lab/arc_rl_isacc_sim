import argparse
import os

os.environ.setdefault("PYTORCH_CUDA_ALLOC_CONF", "expandable_segments:True")

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(
    description="Train SKRL AAC policy for ARCPro Lane Following."
)
parser.add_argument(
    "--num_envs",
    type=int,
    default=20,
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
import sys
import re
import subprocess
import warnings
from datetime import datetime

warnings.filterwarnings("ignore", category=DeprecationWarning)

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))
ARCPRO_LAB_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)
if ARCPRO_LAB_DIR not in sys.path:
    sys.path.insert(0, ARCPRO_LAB_DIR)

from isaaclab.envs import ManagerBasedRLEnv
from arcpro_env_cfg import ARCProEnvCfg

from skrl.envs.wrappers.torch import IsaacLabWrapper
from skrl.agents.torch.ppo import PPO, PPO_DEFAULT_CONFIG
from skrl.trainers.torch import SequentialTrainer
from skrl.memories.torch import RandomMemory
import itertools
import gymnasium as gym
import torch
import torch.nn as nn
import torch.nn.functional as F
from skrl import config
from skrl.resources.schedulers.torch import KLAdaptiveLR

from agents.skrl_wrappers import SKRLFlattenWrapper

# Import the new SKRL models
from agents.skrl_models import ARCProActor, ARCProCritic


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

    def _update(self, timestep: int, timesteps: int) -> None:
        """PPO update with CPU-safe GAE computation and on-the-fly GPU mini-batch streaming."""

        def compute_gae(
            rewards: torch.Tensor,
            dones: torch.Tensor,
            values: torch.Tensor,
            next_values: torch.Tensor,
            discount_factor: float = 0.99,
            lambda_coefficient: float = 0.95,
        ) -> torch.Tensor:
            advantage = 0
            advantages = torch.zeros_like(rewards)
            not_dones = dones.logical_not()
            memory_size = rewards.shape[0]

            # advantages computation
            for i in reversed(range(memory_size)):
                next_val = values[i + 1] if i < memory_size - 1 else next_values
                advantage = (
                    rewards[i]
                    - values[i]
                    + discount_factor
                    * not_dones[i]
                    * (next_val + lambda_coefficient * advantage)
                )
                advantages[i] = advantage
            # returns computation
            returns = advantages + values
            # normalize advantages
            advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)
            return returns, advantages

        # 1. Compute bootstrap values for next states
        with torch.no_grad():
            self.value.train(False)
            last_values, _, _ = self.value.act(
                {"states": self._state_preprocessor(self._current_next_states.float())},
                role="value",
            )
            self.value.train(True)
        last_values = self._value_preprocessor(last_values, inverse=True)

        # 2. DEVICE ALIGNMENT: Transfer last_values to memory device (CPU) for GAE computation
        if self.memory.device != last_values.device:
            last_values = last_values.to(self.memory.device)

        values = self.memory.get_tensor_by_name("values")
        returns, advantages = compute_gae(
            rewards=self.memory.get_tensor_by_name("rewards"),
            dones=self.memory.get_tensor_by_name("terminated"),
            values=values,
            next_values=last_values,
            discount_factor=self._discount_factor,
            lambda_coefficient=self._lambda,
        )

        self.memory.set_tensor_by_name(
            "values", self._value_preprocessor(values, train=True)
        )
        self.memory.set_tensor_by_name(
            "returns", self._value_preprocessor(returns, train=True)
        )
        self.memory.set_tensor_by_name("advantages", advantages)

        # 3. Sample mini-batches from CPU memory (0 MB GPU VRAM consumed)
        sampled_batches = self.memory.sample_all(
            names=self._tensors_names, mini_batches=self._mini_batches
        )

        cumulative_policy_loss = 0
        cumulative_entropy_loss = 0
        cumulative_value_loss = 0

        # 4. Learning epochs with on-the-fly streaming to GPU
        for epoch in range(self._learning_epochs):
            kl_divergences = []

            for batch in sampled_batches:
                # Stream ONLY this active mini-batch across PCIe to GPU
                (
                    sampled_states,
                    sampled_actions,
                    sampled_log_prob,
                    sampled_values,
                    sampled_returns,
                    sampled_advantages,
                ) = [t.to(self.device, non_blocking=True) for t in batch]

                sampled_states = self._state_preprocessor(
                    sampled_states, train=not epoch
                )

                _, next_log_prob, _ = self.policy.act(
                    {"states": sampled_states, "taken_actions": sampled_actions},
                    role="policy",
                )

                # Compute approximate KL divergence
                with torch.no_grad():
                    ratio = next_log_prob - sampled_log_prob
                    kl_divergence = ((torch.exp(ratio) - 1) - ratio).mean()
                    kl_divergences.append(kl_divergence)

                # Early stopping with KL divergence
                if self._kl_threshold and kl_divergence > self._kl_threshold:
                    break

                # Compute entropy loss
                if self._entropy_loss_scale:
                    entropy_loss = (
                        -self._entropy_loss_scale
                        * self.policy.get_entropy(role="policy").mean()
                    )
                else:
                    entropy_loss = 0

                # Compute policy loss
                ratio = torch.exp(next_log_prob - sampled_log_prob)
                surrogate = sampled_advantages * ratio
                surrogate_clipped = sampled_advantages * torch.clip(
                    ratio, 1.0 - self._ratio_clip, 1.0 + self._ratio_clip
                )
                policy_loss = -torch.min(surrogate, surrogate_clipped).mean()

                # Compute value loss
                predicted_values, _, _ = self.value.act(
                    {"states": sampled_states}, role="value"
                )
                if self._clip_predicted_values:
                    predicted_values = sampled_values + torch.clip(
                        predicted_values - sampled_values,
                        min=-self._value_clip,
                        max=self._value_clip,
                    )
                value_loss = self._value_loss_scale * F.mse_loss(
                    sampled_returns, predicted_values
                )

                # Optimization step
                self.optimizer.zero_grad(set_to_none=True)
                (policy_loss + entropy_loss + value_loss).backward()
                if config.torch.is_distributed:
                    self.policy.reduce_parameters()
                    if self.policy is not self.value:
                        self.value.reduce_parameters()
                if self._grad_norm_clip > 0:
                    if self.policy is self.value:
                        nn.utils.clip_grad_norm_(
                            self.policy.parameters(), self._grad_norm_clip
                        )
                    else:
                        nn.utils.clip_grad_norm_(
                            itertools.chain(
                                self.policy.parameters(), self.value.parameters()
                            ),
                            self._grad_norm_clip,
                        )
                self.optimizer.step()

                # Update cumulative losses
                cumulative_policy_loss += policy_loss.item()
                cumulative_value_loss += value_loss.item()
                if self._entropy_loss_scale:
                    cumulative_entropy_loss += entropy_loss.item()

            # Update learning rate
            if self._learning_rate_scheduler:
                if isinstance(self.scheduler, KLAdaptiveLR):
                    kl = torch.tensor(kl_divergences, device=self.device).mean()
                    if config.torch.is_distributed:
                        torch.distributed.all_reduce(
                            kl, op=torch.distributed.ReduceOp.SUM
                        )
                        kl /= config.torch.world_size
                    self.scheduler.step(kl.item())
                else:
                    self.scheduler.step()

        # 5. Explicit GC & VRAM cleanup
        del sampled_batches
        if torch.cuda.is_available():
            torch.cuda.empty_cache()

        # Record tracking metrics
        self.track_data(
            "Loss / Policy loss",
            cumulative_policy_loss / (self._learning_epochs * self._mini_batches),
        )
        self.track_data(
            "Loss / Value loss",
            cumulative_value_loss / (self._learning_epochs * self._mini_batches),
        )
        if self._entropy_loss_scale:
            self.track_data(
                "Loss / Entropy loss",
                cumulative_entropy_loss / (self._learning_epochs * self._mini_batches),
            )
        self.track_data(
            "Policy / Standard deviation",
            self.policy.distribution(role="policy").stddev.mean().item(),
        )
        if self._learning_rate_scheduler:
            self.track_data("Learning / Learning rate", self.scheduler.get_last_lr()[0])

    def post_interaction(self, timestep: int, timesteps: int) -> None:
        super().post_interaction(timestep, timesteps)
        if torch.cuda.is_available():
            torch.cuda.empty_cache()

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
        if self._print_counter >= 2048:
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

    # 4. Define Memory: 512 rollouts stored in Host System RAM (device="cpu")
    # Frees 100% of GPU replay buffer VRAM, allowing 20 parallel camera envs with zero OOM risk!
    memory = RandomMemory(memory_size=512, num_envs=env.num_envs, device="cpu")

    # 5. Define Models
    models = {}
    models["policy"] = ARCProActor(env.observation_space, env.action_space, "cuda:0")

    # Use env.state_space for the Critic if available, fallback to observation_space
    state_space = getattr(env, "state_space", env.observation_space)
    models["value"] = ARCProCritic(state_space, env.action_space, "cuda:0")

    # 6. Configure Agent
    cfg_ppo = PPO_DEFAULT_CONFIG.copy()
    cfg_ppo["rollouts"] = 512
    cfg_ppo["mini_batches"] = (
        16  # Mini-batch size = (512 * 20 / 16) = 640 transitions streamed per backprop
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
        0.005  # Raised from 0.001 for more exploration out of local minima
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
        50000  # Save every 50k timesteps (~2.4 hours)
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
