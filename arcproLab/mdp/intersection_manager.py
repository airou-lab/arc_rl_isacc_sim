"""Traffic signal FSM for the ARCPro smart intersection (V2I node).

Pure PyTorch, no Isaac imports -- unit-testable on CPU. Designed to splice
into RoadManager.update() in place of ``self.go_signals[:] = 1.0``.

Protocol (matches the 12-dim telemetry contract in mdp/observations.py):
    obs slot 1 (go_signal):  1.0 = proceed, 0.0 = stop (yellow counts as stop)
    obs slot 2 (proposed):   time-to-change, normalize with ``cycle_s``

Approach groups: agents are assigned to signal groups (e.g. 0 = NS, 1 = EW).
Exactly one group is active at any time; the FSM cycles
group0 GREEN -> group0 YELLOW -> group1 GREEN -> group1 YELLOW -> ...
Supports N groups, per-group green durations, and actuated green extension
(the bidirectional/uplink hook: call extend_green() when demand is sensed).
"""

import torch

GREEN = 0
YELLOW = 1


class IntersectionManager:
    def __init__(
        self,
        num_envs: int,
        green_s=(8.0, 8.0),
        yellow_s: float = 2.0,
        max_green_s: float = 20.0,
        step_dt: float = 0.02,  # 50 Hz control rate: sim dt 0.002 * decimation 10
        randomize_phase: bool = True,
        device: str = "cpu",
        seed: int | None = None,
    ):
        self.num_envs = num_envs
        self.num_groups = len(green_s)
        self.device = device
        self.step_dt = float(step_dt)
        self.green_s = torch.tensor(green_s, dtype=torch.float32, device=device)
        self.yellow_s = float(yellow_s)
        self.max_green_s = float(max_green_s)
        self.gen = torch.Generator(device="cpu")
        if seed is not None:
            self.gen.manual_seed(seed)

        # Per-env FSM state
        self.group = torch.zeros(num_envs, dtype=torch.long, device=device)
        self.state = torch.full((num_envs,), GREEN, dtype=torch.long, device=device)
        self.t_rem = self.green_s[self.group].clone()
        self.t_elapsed = torch.zeros(num_envs, device=device)

        if randomize_phase:
            self.reset(torch.ones(num_envs, dtype=torch.bool, device=device))

    @property
    def cycle_s(self) -> float:
        """Full cycle duration (for normalizing time-to-change)."""
        return float(self.green_s.sum().item() + self.num_groups * self.yellow_s)

    def reset(self, env_mask: torch.Tensor):
        """Re-randomize signal phase for masked envs (call on episode reset).

        Each reset env starts GREEN on a random group at a random point in
        that green, so agents see varied phases across resets.
        """
        n = int(env_mask.sum().item())
        if n == 0:
            return
        rand_group = torch.randint(
            0, self.num_groups, (n,), generator=self.gen
        ).to(self.device)
        frac = torch.rand(n, generator=self.gen).to(self.device)
        g = self.green_s[rand_group]
        self.group[env_mask] = rand_group
        self.state[env_mask] = GREEN
        self.t_rem[env_mask] = g * (1.0 - frac)  # in (0, g]
        self.t_elapsed[env_mask] = g * frac

    def step(self):
        """Advance the FSM by one control step."""
        self.t_rem -= self.step_dt
        self.t_elapsed += self.step_dt
        expired = self.t_rem <= 0.0
        was_green = self.state == GREEN

        g2y = expired & was_green
        if g2y.any():
            self.state[g2y] = YELLOW
            self.t_rem[g2y] = self.yellow_s
            self.t_elapsed[g2y] = 0.0

        y2g = expired & ~was_green
        if y2g.any():
            nxt = (self.group[y2g] + 1) % self.num_groups
            self.group[y2g] = nxt
            self.state[y2g] = GREEN
            self.t_rem[y2g] = self.green_s[nxt]
            self.t_elapsed[y2g] = 0.0

    def extend_green(self, env_mask: torch.Tensor, ext_s: float):
        """Actuated-control hook (uplink): extend the current green.

        Only applies to envs currently in GREEN; total green is capped at
        ``max_green_s`` so a saturated approach cannot starve the cross street.
        """
        can = env_mask & (self.state == GREEN)
        if not can.any():
            return
        budget = (self.max_green_s - (self.t_elapsed + self.t_rem)).clamp(min=0.0)
        ext = torch.minimum(
            torch.full_like(self.t_rem, float(ext_s)), budget
        )
        self.t_rem = torch.where(can, self.t_rem + ext, self.t_rem)

    def get_spat(self, agent_groups: torch.Tensor):
        """SPaT downlink. agent_groups: (num_envs, num_agents) long tensor
        mapping each agent to its approach group.

        Returns (go, ttc, phase), each (num_envs, num_agents):
            go:    1.0 while the agent's group is GREEN, else 0.0
            ttc:   seconds until the agent's movement permission next changes
                   (green -> end of green; stopped -> start of its next green)
            phase: 0 = green, 1 = yellow, 2 = red (for logging / richer obs)
        """
        G = self.num_groups
        grp = self.group.unsqueeze(1)   # (B,1)
        st = self.state.unsqueeze(1)    # (B,1)
        trem = self.t_rem.unsqueeze(1)  # (B,1)

        active = agent_groups == grp
        green_now = active & (st == GREEN)
        go = green_now.float()

        phase = torch.full_like(agent_groups, 2)
        phase[green_now] = 0
        phase[active & (st == YELLOW)] = 1

        # Stopped agents: finish current state, then walk the ring of
        # intervening groups (each contributes green + yellow) to my green.
        stop_wait = (trem + (st == GREEN).float() * self.yellow_s).expand_as(go).clone()
        delta = (agent_groups - grp - 1) % G  # count of intervening groups
        for j in range(G - 1):
            k = (grp + 1 + j) % G                      # (B,1)
            block = self.green_s[k] + self.yellow_s    # (B,1)
            stop_wait = stop_wait + (delta > j).float() * block

        ttc = torch.where(green_now, trem.expand_as(go), stop_wait)
        return go, ttc, phase


# ---------------------------------------------------------------------------
# Integration sketch (RoadManager.update splice point):
#
#   # __init__:
#   self.intersection = IntersectionManager(num_envs, step_dt=0.02, device=device)
#   self.agent_groups = torch.zeros((num_envs, num_agents), dtype=torch.long,
#                                   device=device)  # TODO: assign per approach
#   self.time_to_change = torch.zeros((num_envs, num_agents), device=device)
#
#   # update(), replacing `self.go_signals[:] = 1.0`:
#   if reset_buf is not None and reset_buf.any():
#       self.intersection.reset(reset_buf)
#   self.intersection.step()
#   go, ttc, _ = self.intersection.get_spat(self.agent_groups)
#   self.go_signals = go
#   self.time_to_change = ttc / self.intersection.cycle_s  # -> obs slot 2
# ---------------------------------------------------------------------------
