from __future__ import annotations

import collections
import copy
import sys
from typing import Any

import numpy as np
from packaging import version
import skrl
from skrl import config, logger
from skrl.agents.torch.ppo import PPO
from skrl.resources.preprocessors.torch import RunningStandardScaler
from skrl.resources.schedulers.torch import KLAdaptiveLR
import torch
import torch.nn as nn
import torch.nn.functional as F

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

PPO_DEFAULT_CONFIG = {
    "rollouts": 16,
    "learning_epochs": 8,
    "mini_batches": 2,
    "discount_factor": 0.99,
    "lambda": 0.95,
    "learning_rate": 3e-4,
    "policy_learning_rate": None,
    "value_learning_rate": None,
    "learning_rate_scheduler": None,
    "learning_rate_scheduler_kwargs": {},
    "state_preprocessor": None,
    "state_preprocessor_kwargs": {},
    "value_preprocessor": RunningStandardScaler,
    "value_preprocessor_kwargs": {"size": 1},
    "random_timesteps": 0,
    "learning_starts": 0,
    "grad_norm_clip": 0.5,
    "ratio_clip": 0.2,
    "value_clip": 0.2,
    "clip_predicted_values": True,
    "entropy_loss_scale": 0.005,
    "value_loss_scale": 0.5,
    "mirror_symmetry_weight": 0.5,
    "kl_threshold": 0.016,
    "rewards_shaper": None,
    "time_limit_bootstrap": True,
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

try:
    import skrl.agents.torch.ppo as _skrl_ppo_mod

    if not hasattr(_skrl_ppo_mod, "PPO_DEFAULT_CONFIG"):
        _skrl_ppo_mod.PPO_DEFAULT_CONFIG = PPO_DEFAULT_CONFIG
except Exception:
    pass


def mirror_observation_batch(
    states: torch.Tensor,
    tel_mirror_signs: torch.Tensor,
    spatial_parity: torch.Tensor | None = None,
) -> torch.Tensor:
    """Apply horizontal mirror operator T_obs to a batch of flattened observations."""
    D = states.shape[1]
    if D >= 25088 + TELEMETRY_DIM:
        vis = states[:, :25088].view(-1, 512, 7, 7)
        vis_mir = torch.flip(vis, dims=[-1]).reshape(-1, 25088)
        tel = states[:, 25088 : 25088 + TELEMETRY_DIM]
        tel_mir = tel * tel_mirror_signs
        if D > 25088 + TELEMETRY_DIM:
            return torch.cat(
                [vis_mir, tel_mir, states[:, 25088 + TELEMETRY_DIM :]], dim=-1
            )
        return torch.cat([vis_mir, tel_mir], dim=-1)
    elif D == 64 + TELEMETRY_DIM:
        k = states[:, :64]
        if spatial_parity is None:
            spatial_parity = torch.tensor(
                [-1.0, 1.0] * 32, device=states.device, dtype=states.dtype
            )
        k_mir = k * spatial_parity
        tel_mir = states[:, 64:] * tel_mirror_signs
        return torch.cat([k_mir, tel_mir], dim=-1)
    elif D >= TELEMETRY_DIM:
        out = states.clone()
        out[:, -TELEMETRY_DIM:] = out[:, -TELEMETRY_DIM:] * tel_mirror_signs
        return out
    return states.clone()


class TelemetryPPO(PPO):
    def __init__(
        self,
        models: dict[str, Any] | None = None,
        memory: Any = None,
        observation_space: Any = None,
        action_space: Any = None,
        device: str | torch.device | None = None,
        cfg: dict[str, Any] | Any = None,
        state_space: Any = None,
        *,
        initial_step: int = 0,
        num_envs: int = 16,
    ):
        raw_cfg = copy.deepcopy(PPO_DEFAULT_CONFIG)
        if isinstance(cfg, dict):
            raw_cfg.update(cfg)
        elif cfg is not None and hasattr(cfg, "__dict__"):
            raw_cfg.update(
                {k: v for k, v in cfg.__dict__.items() if not k.startswith("_")}
            )

        self.mirror_symmetry_weight = float(
            raw_cfg.pop("mirror_symmetry_weight", 0.5)
        )
        base_lr = raw_cfg.get("learning_rate", 3e-4)
        if isinstance(base_lr, (tuple, list)):
            default_pol_lr, default_val_lr = float(base_lr[0]), float(base_lr[-1])
        else:
            default_pol_lr = default_val_lr = float(base_lr)

        pol_lr_cfg = raw_cfg.pop("policy_learning_rate", None)
        val_lr_cfg = raw_cfg.pop("value_learning_rate", None)
        self._policy_lr = (
            float(pol_lr_cfg) if pol_lr_cfg is not None else default_pol_lr
        )
        self._value_lr = (
            float(val_lr_cfg) if val_lr_cfg is not None else default_val_lr
        )
        clip_pred_vals = bool(raw_cfg.get("clip_predicted_values", True))

        # Ensure value_preprocessor_kwargs has size=1 and target device
        resolved_device = (
            torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
            if device is None
            else torch.device(device)
        )
        if raw_cfg.get("value_preprocessor") is None:
            raw_cfg["value_preprocessor"] = RunningStandardScaler
        if raw_cfg.get("value_preprocessor") is False:
            raw_cfg["value_preprocessor"] = None
        if raw_cfg.get("value_preprocessor") is not None:
            vp_kwargs = dict(raw_cfg.get("value_preprocessor_kwargs") or {})
            vp_kwargs.setdefault("size", 1)
            vp_kwargs.setdefault("device", resolved_device)
            raw_cfg["value_preprocessor_kwargs"] = vp_kwargs

        if _SKRL_V2:
            from skrl.agents.torch.ppo import PPO_CFG
            import dataclasses

            v2_dict = copy.deepcopy(raw_cfg)
            if "lambda" in v2_dict and "gae_lambda" not in v2_dict:
                v2_dict["gae_lambda"] = v2_dict.pop("lambda")
            valid_fields = {f.name for f in dataclasses.fields(PPO_CFG)}
            v2_filtered = {k: v for k, v in v2_dict.items() if k in valid_fields}
            super().__init__(
                models=models,
                memory=memory,
                observation_space=observation_space,
                state_space=state_space,
                action_space=action_space,
                device=resolved_device,
                cfg=v2_filtered,
            )
        else:
            super().__init__(
                models=models,
                memory=memory,
                observation_space=observation_space,
                action_space=action_space,
                device=resolved_device,
                cfg=raw_cfg,
            )

        # Harmonize internal attributes across skrl 1.3.0 and skrl 2.1.0
        self._learning_epochs = int(raw_cfg.get("learning_epochs", 8))
        self._mini_batches = int(raw_cfg.get("mini_batches", 2))
        self._rollouts = int(raw_cfg.get("rollouts", 16))
        self._rollout = 0
        self._grad_norm_clip = float(raw_cfg.get("grad_norm_clip", 0.5))
        self._ratio_clip = float(raw_cfg.get("ratio_clip", 0.2))
        self._value_clip = float(raw_cfg.get("value_clip", 0.2))
        self._clip_predicted_values = clip_pred_vals and (self._value_clip > 0.0)
        self._value_loss_scale = float(raw_cfg.get("value_loss_scale", 0.5))
        self._entropy_loss_scale = float(raw_cfg.get("entropy_loss_scale", 0.005))
        self._kl_threshold = float(raw_cfg.get("kl_threshold", 0.016))
        self._learning_rate = self._policy_lr
        self._learning_rate_scheduler = raw_cfg.get("learning_rate_scheduler", None)
        self._discount_factor = float(raw_cfg.get("discount_factor", 0.99))
        self._lambda = float(
            raw_cfg.get("lambda", raw_cfg.get("gae_lambda", 0.95))
        )
        self._random_timesteps = int(raw_cfg.get("random_timesteps", 0))
        self._learning_starts = int(raw_cfg.get("learning_starts", 0))
        self._rewards_shaper = raw_cfg.get("rewards_shaper", None)
        self._time_limit_bootstrap = bool(
            raw_cfg.get("time_limit_bootstrap", True)
        )

        # Decoupled Actor & Critic Optimizers so Critic value spikes never attenuate Policy gradients
        if self.policy is not None and self.value is not None:
            self.policy_optimizer = torch.optim.Adam(
                self.policy.parameters(), lr=self._policy_lr
            )
            self.value_optimizer = torch.optim.Adam(
                self.value.parameters(), lr=self._value_lr
            )
            # Keep self.optimizer pointing to policy_optimizer for backward compatibility with external scripts
            self.optimizer = self.policy_optimizer
            self.checkpoint_modules["policy_optimizer"] = self.policy_optimizer
            self.checkpoint_modules["value_optimizer"] = self.value_optimizer
            self.checkpoint_modules["optimizer"] = self.policy_optimizer

            self.policy_scheduler = None
            self.value_scheduler = None
            self.scheduler = None
            sched_cls = self._learning_rate_scheduler
            if isinstance(sched_cls, (tuple, list)):
                sched_cls = sched_cls[0]
            sched_kwargs = raw_cfg.get("learning_rate_scheduler_kwargs", {})
            if isinstance(sched_kwargs, (tuple, list)):
                sched_kwargs = sched_kwargs[0]
            if sched_cls is not None:
                self.policy_scheduler = sched_cls(
                    self.policy_optimizer, **(sched_kwargs or {})
                )
                self.value_scheduler = sched_cls(
                    self.value_optimizer, **(sched_kwargs or {})
                )
                self.scheduler = self.policy_scheduler

        self.initial_step = initial_step
        self.num_envs = num_envs
        self._print_counter = 0
        self._last_ep_len = 0.0
        self._last_ep_len_max = 0.0
        self._last_ep_rew = 0.0
        self._last_ep_rew_max = 0.0

        # Window-immune persistent episode telemetry deques (never cleared by write_tracking_data)
        self._ep_rew_window: collections.deque[float] = collections.deque(maxlen=100)
        self._ep_len_window: collections.deque[int] = collections.deque(maxlen=100)
        self._ep_cum_rewards: torch.Tensor | None = None
        self._ep_cum_lengths: torch.Tensor | None = None

        self._tel_mirror_signs = torch.tensor(
            TELEMETRY_MIRROR_SIGNS, dtype=torch.float32, device=self.device
        )
        self._act_mirror_signs = torch.tensor(
            ACTION_MIRROR_SIGNS, dtype=torch.float32, device=self.device
        )
        self._spatial_parity = torch.tensor(
            [-1.0, 1.0] * 32, dtype=torch.float32, device=self.device
        )

    def set_mode(self, mode: str) -> None:
        is_train = mode == "train"
        self.training = is_train
        for model in self.models.values():
            if model is not None:
                if hasattr(model, "set_mode"):
                    model.set_mode(mode)
                else:
                    model.train(is_train)

    def init(self, trainer_cfg: dict[str, Any] | None = None) -> None:
        super().init(trainer_cfg=trainer_cfg)
        self.set_mode("eval")

        if self.memory is not None:
            existing = set(self.memory.get_tensor_names())
            tensor_specs = [
                ("states", self.observation_space, torch.float32),
                ("actions", self.action_space, torch.float32),
                ("rewards", 1, torch.float32),
                ("terminated", 1, torch.bool),
                ("truncated", 1, torch.bool),
                ("log_prob", 1, torch.float32),
                ("values", 1, torch.float32),
                ("returns", 1, torch.float32),
                ("advantages", 1, torch.float32),
            ]
            for t_name, t_size, t_dtype in tensor_specs:
                if t_name not in existing:
                    self.memory.create_tensor(name=t_name, size=t_size, dtype=t_dtype)

            self._tensors_names = [
                "states",
                "actions",
                "log_prob",
                "values",
                "returns",
                "advantages",
            ]

        self._current_log_prob = None
        self._current_next_states = None

    def load(self, path: str) -> None:
        """Load checkpoint modules while gracefully handling legacy single-optimizer checkpoints."""
        if version.parse(torch.__version__) >= version.parse("1.13"):
            modules = torch.load(path, map_location=self.device, weights_only=False)
        else:
            modules = torch.load(path, map_location=self.device)

        if isinstance(modules, dict):
            for name, data in modules.items():
                module = self.checkpoint_modules.get(name, None)
                if module is not None and hasattr(module, "load_state_dict"):
                    try:
                        module.load_state_dict(data)
                        if hasattr(module, "eval"):
                            module.eval()
                    except Exception as exc:
                        logger.warning(
                            f"Skipping incompatible state_dict for module '{name}': {exc}"
                        )
                else:
                    logger.warning(
                        f"Skipping module '{name}'. The agent doesn't have such an instance"
                    )

    def _offset_timesteps(self, timestep: int | None, timesteps: int | None):
        cum_timestep = (timestep + self.initial_step) if timestep is not None else None
        cum_timesteps = (
            (timesteps + self.initial_step) if timesteps is not None else None
        )
        return cum_timestep, cum_timesteps

    def act(
        self,
        states: torch.Tensor,
        *args: Any,
        timestep: int = 0,
        timesteps: int = 0,
        **kwargs: Any,
    ):
        """Support both skrl 1.3.0 act(states, timestep, timesteps) and skrl 2.1.0 act(observations, states, *, timestep, timesteps)."""
        called_v2_signature = False
        if len(args) == 2 and isinstance(args[0], int) and isinstance(args[1], int):
            timestep, timesteps = args[0], args[1]
        elif len(args) >= 1 and (args[0] is None or isinstance(args[0], torch.Tensor)):
            called_v2_signature = True

        proc_states = self._state_preprocessor(states)
        if timestep < self._random_timesteps:
            res = self.policy.random_act({"states": proc_states}, role="policy")
            if called_v2_signature and len(res) == 3:
                return res[0], res[2]
            return res

        actions, log_prob, outputs = self.policy.act(
            {"states": proc_states}, role="policy"
        )
        self._current_log_prob = log_prob
        outputs["log_prob"] = log_prob

        if called_v2_signature:
            return actions, outputs
        return actions, log_prob, outputs

    def update(self, *, timestep: int, timesteps: int) -> None:
        self._update(timestep=timestep, timesteps=timesteps)

    def _update(self, timestep: int, timesteps: int) -> None:
        """PPO update with CPU-safe GAE, truncated bootstrapping, decoupled Actor/Critic optimizers, and mirror symmetry loss."""

        def compute_gae(
            rewards: torch.Tensor,
            terminated: torch.Tensor,
            truncated: torch.Tensor,
            values: torch.Tensor,
            next_values: torch.Tensor,
            discount_factor: float = 0.99,
            lambda_coefficient: float = 0.95,
        ) -> tuple[torch.Tensor, torch.Tensor]:
            advantage = torch.zeros_like(rewards[0])
            advantages = torch.zeros_like(rewards)
            not_terminated = terminated.logical_not().to(dtype=rewards.dtype)
            not_done = (terminated | truncated).logical_not().to(dtype=rewards.dtype)
            memory_size = rewards.shape[0]

            for i in reversed(range(memory_size)):
                next_val = values[i + 1] if i < memory_size - 1 else next_values
                # Bootstrap future value when not_terminated (including truncated timeout steps),
                # while resetting GAE lambda trace across any episode boundary (terminated | truncated).
                delta = (
                    rewards[i]
                    + discount_factor * not_terminated[i] * next_val
                    - values[i]
                )
                advantage = (
                    delta
                    + discount_factor * lambda_coefficient * not_done[i] * advantage
                )
                advantages[i] = advantage

            returns = advantages + values
            advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)
            return returns, advantages

        # 1. Compute bootstrap values for next states
        with torch.no_grad():
            self.value.train(False)
            next_states_dev = self._current_next_states.to(
                device=self.device, dtype=torch.float32
            )
            last_values, _, _ = self.value.act(
                {"states": self._state_preprocessor(next_states_dev)},
                role="value",
            )
            self.value.train(True)
            last_values = self._value_preprocessor(
                last_values.to(device=self.device), inverse=True
            )

        if self.memory.device != last_values.device:
            last_values = last_values.to(self.memory.device)

        values = self.memory.get_tensor_by_name("values")
        terminated_tensor = self.memory.get_tensor_by_name("terminated")
        try:
            truncated_tensor = self.memory.get_tensor_by_name("truncated")
        except KeyError:
            truncated_tensor = torch.zeros_like(terminated_tensor)

        returns, advantages = compute_gae(
            rewards=self.memory.get_tensor_by_name("rewards"),
            terminated=terminated_tensor,
            truncated=truncated_tensor,
            values=values,
            next_values=last_values,
            discount_factor=self._discount_factor,
            lambda_coefficient=self._lambda,
        )

        # Standardize values and returns via value_preprocessor on agent device, then copy back to memory
        norm_values = self._value_preprocessor(
            values.to(device=self.device), train=True
        ).to(device=self.memory.device)
        norm_returns = self._value_preprocessor(
            returns.to(device=self.device), train=True
        ).to(device=self.memory.device)

        self.memory.set_tensor_by_name("values", norm_values)
        self.memory.set_tensor_by_name("returns", norm_returns)
        self.memory.set_tensor_by_name("advantages", advantages)

        # Update Critic 12D telemetry running scaler once per rollout on valid states
        if hasattr(self.value, "update_telemetry_stats"):
            with torch.no_grad():
                all_states_view = self.memory.tensors_view["states"][: len(self.memory)]
                all_tel = all_states_view[:, -TELEMETRY_DIM:].to(device=self.device)
                self.value.update_telemetry_stats(all_tel)

        cumulative_policy_loss = 0.0
        cumulative_entropy_loss = 0.0
        cumulative_value_loss = 0.0
        cumulative_sym_loss = 0.0
        cumulative_pol_grad_norm = 0.0
        cumulative_val_grad_norm = 0.0
        total_batches_executed = 0
        early_stop = False

        total_samples = len(self.memory)
        tel_signs = self._tel_mirror_signs.to(device=self.device)
        act_signs = self._act_mirror_signs.to(device=self.device)
        spatial_parity = self._spatial_parity.to(device=self.device)

        # 4. Learning epochs with per-epoch shuffled mini-batches streamed on-the-fly to GPU
        for epoch in range(self._learning_epochs):
            kl_divergences = []

            perm = np.random.permutation(total_samples)
            sampled_batches = self.memory.sample_by_index(
                names=self._tensors_names,
                indexes=perm,
                mini_batches=self._mini_batches,
            )

            for batch in sampled_batches:
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

                _, next_log_prob, out_orig = self.policy.act(
                    {"states": sampled_states, "taken_actions": sampled_actions},
                    role="policy",
                )

                # Compute approximate KL divergence
                with torch.no_grad():
                    log_ratio = next_log_prob - sampled_log_prob
                    kl_divergence = ((torch.exp(log_ratio) - 1) - log_ratio).mean()
                    kl_divergences.append(kl_divergence)

                # Compute entropy loss
                if self._entropy_loss_scale:
                    entropy_loss = (
                        -self._entropy_loss_scale
                        * self.policy.get_entropy(role="policy").mean()
                    )
                else:
                    entropy_loss = torch.tensor(0.0, device=self.device)

                # Compute clipped PPO policy surrogate loss
                ratio = torch.exp(next_log_prob - sampled_log_prob)
                surrogate = sampled_advantages * ratio
                surrogate_clipped = sampled_advantages * torch.clip(
                    ratio, 1.0 - self._ratio_clip, 1.0 + self._ratio_clip
                )
                policy_loss = -torch.min(surrogate, surrogate_clipped).mean()

                # Compute Mirror Symmetry Loss L_sym
                if self.mirror_symmetry_weight > 0.0:
                    states_mirror = mirror_observation_batch(
                        sampled_states, tel_signs, spatial_parity
                    )
                    mu_orig = out_orig.get("mean_actions", None)
                    if mu_orig is None:
                        mu_orig, _, out_orig = self.policy.compute(
                            {"states": sampled_states}, role="policy"
                        )
                    mu_mirror, _, out_mirror = self.policy.compute(
                        {"states": states_mirror}, role="policy"
                    )
                    sym_mu = F.mse_loss(
                        mu_mirror, act_signs * mu_orig.detach()
                    ) + F.mse_loss(mu_orig, act_signs * mu_mirror.detach())
                    if "raw_logits" in out_orig and "raw_logits" in out_mirror:
                        raw_orig_act = torch.tanh(out_orig["raw_logits"])
                        raw_mir_act = torch.tanh(out_mirror["raw_logits"])
                        sym_raw = F.mse_loss(
                            raw_mir_act, act_signs * raw_orig_act.detach()
                        ) + F.mse_loss(raw_orig_act, act_signs * raw_mir_act.detach())
                        sym_loss = self.mirror_symmetry_weight * 0.25 * (sym_mu + sym_raw)
                    else:
                        sym_loss = self.mirror_symmetry_weight * 0.5 * sym_mu
                else:
                    sym_loss = torch.tensor(0.0, device=self.device)

                # Compute Critic value loss (with value clipping in normalized return space)
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

                # Decoupled Optimization Step 1: Policy (Actor)
                self.policy_optimizer.zero_grad(set_to_none=True)
                (policy_loss + entropy_loss + sym_loss).backward()
                if config.torch.is_distributed:
                    self.policy.reduce_parameters()
                pol_grad_norm = float(
                    nn.utils.clip_grad_norm_(
                        self.policy.parameters(),
                        self._grad_norm_clip if self._grad_norm_clip > 0 else float("inf"),
                    ).item()
                )
                self.policy_optimizer.step()

                # Decoupled Optimization Step 2: Value (Critic)
                self.value_optimizer.zero_grad(set_to_none=True)
                value_loss.backward()
                if config.torch.is_distributed and self.policy is not self.value:
                    self.value.reduce_parameters()
                val_grad_norm = float(
                    nn.utils.clip_grad_norm_(
                        self.value.parameters(),
                        self._grad_norm_clip if self._grad_norm_clip > 0 else float("inf"),
                    ).item()
                )
                self.value_optimizer.step()

                cumulative_policy_loss += float(policy_loss.detach().item())
                cumulative_value_loss += float(value_loss.detach().item())
                cumulative_sym_loss += float(sym_loss.detach().item())
                cumulative_pol_grad_norm += pol_grad_norm
                cumulative_val_grad_norm += val_grad_norm
                if self._entropy_loss_scale:
                    cumulative_entropy_loss += float(entropy_loss.detach().item())
                total_batches_executed += 1

                if (
                    self._kl_threshold
                    and float(kl_divergence.item()) > 2.5 * self._kl_threshold
                ):
                    early_stop = True
                    break

            # Update learning rate schedulers if configured
            if self._learning_rate_scheduler and self.policy_scheduler is not None:
                if isinstance(self.policy_scheduler, KLAdaptiveLR):
                    kl = torch.stack(kl_divergences).mean()
                    if config.torch.is_distributed:
                        torch.distributed.all_reduce(
                            kl, op=torch.distributed.ReduceOp.SUM
                        )
                        kl /= config.torch.world_size
                    kl_val = float(kl.item())
                    self.policy_scheduler.step(kl_val)
                    if self.value_scheduler is not None:
                        self.value_scheduler.step(kl_val)
                else:
                    self.policy_scheduler.step()
                    if self.value_scheduler is not None:
                        self.value_scheduler.step()

            epoch_mean_kl = (
                float(torch.stack(kl_divergences).mean().item())
                if kl_divergences
                else 0.0
            )
            if early_stop or (
                self._kl_threshold and epoch_mean_kl > self._kl_threshold
            ):
                break

        if torch.cuda.is_available():
            torch.cuda.empty_cache()

        divisor = max(total_batches_executed, 1)
        self.track_data("Loss / Policy loss", cumulative_policy_loss / divisor)
        self.track_data("Loss / Value loss", cumulative_value_loss / divisor)
        self.track_data("Loss / Mirror symmetry", cumulative_sym_loss / divisor)
        self.track_data("Policy / Grad Norm (Actor)", cumulative_pol_grad_norm / divisor)
        self.track_data("Policy / Grad Norm (Critic)", cumulative_val_grad_norm / divisor)
        if self._entropy_loss_scale:
            self.track_data("Loss / Entropy loss", cumulative_entropy_loss / divisor)
        dist = self.policy.distribution(role="policy")
        if dist is not None:
            self.track_data(
                "Policy / Standard deviation", float(dist.stddev.mean().item())
            )
        if self._learning_rate_scheduler and self.policy_scheduler is not None:
            self.track_data(
                "Learning / Learning rate", self.policy_scheduler.get_last_lr()[0]
            )

    def post_interaction(self, timestep: int, timesteps: int) -> None:
        self._rollout += 1
        if not self._rollout % self._rollouts and timestep >= self._learning_starts:
            self.set_mode("train")
            self._update(timestep, timesteps)
            self.set_mode("eval")

        # Write tracking data and checkpoints via Agent.post_interaction
        if _SKRL_V2:
            super(PPO, self).post_interaction(timestep=timestep, timesteps=timesteps)
        else:
            super(PPO, self).post_interaction(timestep, timesteps)

        if torch.cuda.is_available():
            torch.cuda.empty_cache()

    def _sync_persistent_episode_tracking(self) -> None:
        if len(self._ep_rew_window) > 0:
            if not self.tracking_data.get("Reward / Total reward (mean)"):
                self.tracking_data["Reward / Total reward (mean)"].append(
                    self._last_ep_rew
                )
                self.tracking_data["Reward / Total reward (max)"].append(
                    self._last_ep_rew_max
                )
                self.tracking_data["Reward / Total reward (min)"].append(
                    float(np.min(self._ep_rew_window))
                )
                self.tracking_data["Episode / Total timesteps (mean)"].append(
                    self._last_ep_len
                )
                self.tracking_data["Episode / Total timesteps (max)"].append(
                    self._last_ep_len_max
                )
                self.tracking_data["Episode / Total timesteps (min)"].append(
                    float(np.min(self._ep_len_window))
                )

    def write_tracking_data(self, timestep: int, timesteps: int) -> None:
        self._sync_persistent_episode_tracking()
        empty_keys = [k for k, v in self.tracking_data.items() if len(v) == 0]
        for k in empty_keys:
            del self.tracking_data[k]
        cum_ts, cum_total = self._offset_timesteps(timestep, timesteps)
        if _SKRL_V2:
            super().write_tracking_data(timestep=cum_ts, timesteps=cum_total)
        else:
            super().write_tracking_data(cum_ts, cum_total)
        self._sync_persistent_episode_tracking()

    def write_checkpoint(self, timestep: int, timesteps: int) -> None:
        cum_ts, cum_total = self._offset_timesteps(timestep, timesteps)
        if _SKRL_V2:
            super().write_checkpoint(timestep=cum_ts, timesteps=cum_total)
        else:
            super().write_checkpoint(cum_ts, cum_total)

    def record_transition(
        self,
        states: torch.Tensor | None = None,
        actions: torch.Tensor | None = None,
        rewards: torch.Tensor | None = None,
        next_states: torch.Tensor | None = None,
        terminated: torch.Tensor | None = None,
        truncated: torch.Tensor | None = None,
        infos: Any = None,
        timestep: int = 0,
        timesteps: int = 0,
        *,
        observations: torch.Tensor | None = None,
        next_observations: torch.Tensor | None = None,
    ) -> None:
        if states is None and observations is not None:
            states = observations
        if next_states is None and next_observations is not None:
            next_states = next_observations
        if infos is None:
            infos = {}

        cum_timestep, cum_timesteps = self._offset_timesteps(timestep, timesteps)

        # 1. Persistent GPU-accumulated episode rewards & lengths (immune to write_interval clears)
        if rewards is not None and terminated is not None and truncated is not None:
            with torch.no_grad():
                r_vec = rewards.view(-1).to(dtype=torch.float32)
                done_vec = (terminated.view(-1) | truncated.view(-1)).to(
                    dtype=torch.bool
                )
                if (
                    self._ep_cum_rewards is None
                    or self._ep_cum_rewards.shape != r_vec.shape
                    or self._ep_cum_rewards.device != r_vec.device
                ):
                    self._ep_cum_rewards = torch.zeros_like(
                        r_vec, dtype=torch.float32
                    )
                    self._ep_cum_lengths = torch.zeros_like(r_vec, dtype=torch.int32)

                self._ep_cum_rewards.add_(r_vec)
                self._ep_cum_lengths.add_(1)

                if done_vec.any():
                    done_idx = done_vec.nonzero(as_tuple=True)[0]
                    self._ep_rew_window.extend(
                        self._ep_cum_rewards[done_idx].cpu().tolist()
                    )
                    self._ep_len_window.extend(
                        self._ep_cum_lengths[done_idx].cpu().tolist()
                    )
                    self._ep_cum_rewards[done_idx] = 0.0
                    self._ep_cum_lengths[done_idx] = 0

                if len(self._ep_rew_window) > 0:
                    self._last_ep_rew = float(np.mean(self._ep_rew_window))
                    self._last_ep_rew_max = float(np.max(self._ep_rew_window))
                    self._last_ep_len = float(np.mean(self._ep_len_window))
                    self._last_ep_len_max = float(np.max(self._ep_len_window))

        # 2. Store transition in memory
        if self.memory is not None and states is not None:
            self._current_next_states = next_states

            shaped_rewards = rewards
            if self._rewards_shaper is not None:
                shaped_rewards = self._rewards_shaper(rewards, timestep, timesteps)

            with torch.no_grad():
                values, _, _ = self.value.act(
                    {"states": self._state_preprocessor(states.to(self.device))},
                    role="value",
                )
                values = self._value_preprocessor(values, inverse=True)

            self.memory.add_samples(
                states=states,
                actions=actions,
                rewards=shaped_rewards,
                next_states=next_states,
                terminated=terminated,
                truncated=truncated,
                log_prob=self._current_log_prob,
                values=values,
            )

        # 3. Defer .item() GPU-CPU synchronizations strictly to logging/print intervals
        batch_envs = states.shape[0] if states is not None else self.num_envs
        self._print_counter += batch_envs
        write_int = (
            self.write_interval
            if isinstance(self.write_interval, int) and self.write_interval > 0
            else 100
        )
        is_write_step = ((timestep + 1) % write_int == 0)
        is_print_step = self._print_counter >= 2048

        if is_write_step or is_print_step:
            if len(self._ep_rew_window) > 0:
                self.tracking_data["Reward / Total reward (mean)"].append(
                    self._last_ep_rew
                )
                self.tracking_data["Reward / Total reward (max)"].append(
                    self._last_ep_rew_max
                )
                self.tracking_data["Reward / Total reward (min)"].append(
                    float(np.min(self._ep_rew_window))
                )
                self.tracking_data["Episode / Total timesteps (mean)"].append(
                    self._last_ep_len
                )
                self.tracking_data["Episode / Total timesteps (max)"].append(
                    self._last_ep_len_max
                )
                self.tracking_data["Episode / Total timesteps (min)"].append(
                    float(np.min(self._ep_len_window))
                )
            elif self._ep_cum_rewards is not None and self._ep_cum_rewards.numel() > 0:
                # Fallback to active episode accumulators before the first episode finishes
                self._last_ep_rew = float(self._ep_cum_rewards.mean().item())
                self._last_ep_rew_max = float(self._ep_cum_rewards.max().item())
                self._last_ep_len = float(self._ep_cum_lengths.float().mean().item())
                self._last_ep_len_max = float(self._ep_cum_lengths.max().item())

            speed = 0.0
            wp_delta = 0.0
            wp_step = 0.0
            if "speed_mps" in infos:
                speed = float(infos["speed_mps"].float().mean().item())
                self.track_data("Telemetry / Speed_MPS", speed)
            if "dist_white" in infos:
                self.track_data(
                    "Telemetry / Dist_White_m",
                    float(infos["dist_white"].float().mean().item()),
                )
            if "dist_yellow" in infos:
                self.track_data(
                    "Telemetry / Dist_Yellow_m",
                    float(infos["dist_yellow"].float().mean().item()),
                )
            if "track_progress_pct" in infos:
                self.track_data(
                    "Telemetry / Track_Progress_Pct",
                    float(infos["track_progress_pct"].float().mean().item()),
                )
            if "track_wp_delta" in infos:
                wp_delta = float(infos["track_wp_delta"].float().mean().item())
                self.track_data("Telemetry / WP_Delta_Cumulative", wp_delta)
            if "reward_wp_delta_step" in infos:
                wp_step = float(infos["reward_wp_delta_step"].float().mean().item())
                self.track_data("Telemetry / WP_Delta_Step_Reward", wp_step)

            if is_print_step:
                total_env_samples = cum_timestep * self.num_envs
                print(
                    f"Step {cum_timestep:7d} ({total_env_samples / 1e6:5.1f}M env) | Rew: {self._last_ep_rew:6.1f} (max {self._last_ep_rew_max:6.1f}) | "
                    f"Len: {self._last_ep_len:3.0f} (max {self._last_ep_len_max:3.0f}) | "
                    f"Spd: {speed:.2f} | WPs_cum: {wp_delta:.0f} | WPΔ_rew: {wp_step:.2f}"
                )
                sys.stdout.flush()
                self._print_counter = 0
