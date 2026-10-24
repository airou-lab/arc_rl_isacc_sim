import itertools
import sys

import torch
import torch.nn as nn
import torch.nn.functional as F
from skrl import config
from skrl.agents.torch.ppo import PPO
from skrl.resources.schedulers.torch import KLAdaptiveLR


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

    def _offset_timesteps(self, timestep: int | None, timesteps: int | None):
        cum_timestep = (timestep + self.initial_step) if timestep is not None else None
        cum_timesteps = (
            (timesteps + self.initial_step) if timesteps is not None else None
        )
        return cum_timestep, cum_timesteps

    def _update(self, timestep: int, timesteps: int) -> None:
        """PPO update with CPU-safe GAE computation and on-the-fly GPU mini-batch streaming."""

        def compute_gae(
            rewards: torch.Tensor,
            dones: torch.Tensor,
            values: torch.Tensor,
            next_values: torch.Tensor,
            discount_factor: float = 0.99,
            lambda_coefficient: float = 0.95,
        ) -> tuple[torch.Tensor, torch.Tensor]:
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
        total_batches_executed = 0
        early_stop = False

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
                total_batches_executed += 1

                # Early stopping on extreme divergence spike (>2x threshold)
                if (
                    self._kl_threshold
                    and kl_divergence.item() > 2.0 * self._kl_threshold
                ):
                    early_stop = True
                    break

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

            # Epoch-level early stopping: if mean KL across mini-batches exceeds threshold, stop further epochs
            epoch_mean_kl = (
                torch.stack(kl_divergences).mean().item() if kl_divergences else 0.0
            )
            if early_stop or (
                self._kl_threshold and epoch_mean_kl > self._kl_threshold
            ):
                break

        # 5. Explicit GC & VRAM cleanup
        del sampled_batches
        if torch.cuda.is_available():
            torch.cuda.empty_cache()

        # Record tracking metrics normalized by actual batches executed
        divisor = max(total_batches_executed, 1)
        self.track_data(
            "Loss / Policy loss",
            cumulative_policy_loss / divisor,
        )
        self.track_data(
            "Loss / Value loss",
            cumulative_value_loss / divisor,
        )
        if self._entropy_loss_scale:
            self.track_data(
                "Loss / Entropy loss",
                cumulative_entropy_loss / divisor,
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
        super().write_tracking_data(*self._offset_timesteps(timestep, timesteps))

    def write_checkpoint(self, timestep: int, timesteps: int) -> None:
        super().write_checkpoint(*self._offset_timesteps(timestep, timesteps))

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
        cum_timestep, cum_timesteps = self._offset_timesteps(timestep, timesteps)
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
            self.track_data(
                "Telemetry / Track_Progress_Pct",
                infos["track_progress_pct"].mean().item(),
            )
        if "track_wp_delta" in infos:
            # Cumulative WPs from spawn this episode (obs-layer variable)
            wp_delta = infos["track_wp_delta"].float().mean().item()
            self.track_data("Telemetry / WP_Delta_Cumulative", wp_delta)
        # Log the ACTUAL per-step reward delta (rewards.py variable)
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
