"""V2I downlink message buffer: latency, dropout, and beacon-rate model.

Route SPaT messages through this from day one so that ideal comms
(latency 0, dropout 0, beacon every step) is just the degenerate config,
and comms-realism experiments are a knob rather than a refactor.

Real-world anchor: BSM/SPaT beacons at ~10 Hz maps to beacon_interval=5
at the 50 Hz control rate.

Semantics per step(msg) call:
  1. msg is pushed into a ring buffer (the "radio in flight").
  2. The message pushed ``latency_steps`` calls ago becomes deliverable.
  3. Delivery is attempted only on beacon ticks (every ``beacon_interval``
     calls); each attempt independently fails per-env with prob dropout_p.
  4. The receiver holds the last successfully delivered message. ``age``
     counts steps since last delivery; ``valid`` is False if never
     delivered, or if age > stale_after (when stale_after is set).
"""

import torch


class V2IMessageBuffer:
    def __init__(
        self,
        num_envs: int,
        msg_dim: int,
        latency_steps: int = 0,
        dropout_p: float = 0.0,
        beacon_interval: int = 1,
        stale_after: int | None = None,
        device: str = "cpu",
        seed: int | None = None,
    ):
        assert latency_steps >= 0 and beacon_interval >= 1
        self.num_envs = num_envs
        self.msg_dim = msg_dim
        self.latency = latency_steps
        self.depth = latency_steps + 1
        self.dropout_p = float(dropout_p)
        self.beacon_interval = int(beacon_interval)
        self.stale_after = stale_after
        self.device = device
        self.gen = torch.Generator(device="cpu")
        if seed is not None:
            self.gen.manual_seed(seed)

        self.ring = torch.zeros(self.depth, num_envs, msg_dim, device=device)
        self.ring_valid = torch.zeros(self.depth, num_envs, dtype=torch.bool, device=device)
        self.head = 0
        self.step_count = 0

        self.held = torch.zeros(num_envs, msg_dim, device=device)
        self.age = torch.full((num_envs,), float("inf"), device=device)

    def reset(self, env_mask: torch.Tensor):
        """Clear receiver state and in-flight messages for masked envs."""
        self.held[env_mask] = 0.0
        self.age[env_mask] = float("inf")
        self.ring_valid[:, env_mask] = False

    def step(self, msg: torch.Tensor):
        """Push this step's broadcast; return what the receiver currently has.

        Returns (held, valid, age):
            held:  (num_envs, msg_dim) last successfully delivered message
            valid: (num_envs,) bool -- see class docstring
            age:   (num_envs,) float steps since last delivery (inf if never)
        """
        assert msg.shape == (self.num_envs, self.msg_dim), msg.shape

        self.ring[self.head] = msg
        self.ring_valid[self.head] = True
        delayed_idx = (self.head - self.latency) % self.depth
        delayed = self.ring[delayed_idx]
        delayed_ok = self.ring_valid[delayed_idx]
        self.head = (self.head + 1) % self.depth

        self.age = self.age + 1.0
        beacon_tick = (self.step_count % self.beacon_interval) == 0
        self.step_count += 1

        if beacon_tick:
            drop = (
                torch.rand(self.num_envs, generator=self.gen) < self.dropout_p
            ).to(self.device)
            ok = delayed_ok & ~drop
            if ok.any():
                self.held[ok] = delayed[ok]
                self.age[ok] = 0.0

        if self.stale_after is None:
            valid = torch.isfinite(self.age)
        else:
            valid = self.age <= float(self.stale_after)
        return self.held.clone(), valid, self.age.clone()
