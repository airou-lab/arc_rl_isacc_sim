"""SignalizedScheduler -- traffic-light arbitration behind the scheduler API.

Architecture rationale (branch aaron/v2i_intersection):
    The intersection's arbitration authority is pluggable behind one
    interface -- the SchedulerCore-shaped API from the policy repo:

        register_intent(agent_id, intersection_id, turn_command, ...) -> float
        query_go_signal(agent_id, intersection_id, ...)               -> float
        clear_agent(agent_id)                                         -> None
        tick()                                                        -> None
        active_intents                                                -> dict

    The policy repo's IntersectionNodeServer implements RESERVATION-based
    arbitration (conflict-matrix grants). This class implements SIGNALIZED
    arbitration (phase FSM / traffic light) behind the same API, so the two
    paradigms are swappable backends -- and comparable under identical comms
    degradation via V2IMessageBuffer, which wraps the transport layer
    regardless of authority.

    register_intent doubles as the actuation uplink: a registered intent is
    standing demand for its approach group, and tick() extends the active
    green (capped at max_green_s) while demand is present near phase end.
    A vehicle that registers is collaborating; one that never registers
    still gets correct (fixed-time) signals -- graceful degradation when
    the uplink is lost.

    Duck-typed, not subclassed: SchedulerCore lives on policy main, while
    the sim's submodule pin may be on an older lineage. This adapter must
    not depend on which submodule commit is checked out. Once the pin
    advances, subclassing is a two-line change.

    Fail-safe semantics: unknown agents and yellow phases query as 0.0
    (stop). Permission is granted only during green.

Scalar, per-intersection, CPU -- matches the single-process training model
in the policy repo's design. The batched-tensor path for vectorized envs
uses IntersectionManager directly.
"""

import torch

try:
    from mdp.intersection_manager import IntersectionManager, GREEN
except ImportError:  # tests import from a flat path
    from intersection_manager import IntersectionManager, GREEN


class SignalizedScheduler:
    def __init__(
        self,
        green_s=(8.0, 8.0),
        yellow_s: float = 2.0,
        max_green_s: float = 20.0,
        step_dt: float = 0.02,
        actuation_ext_s: float = 2.0,
        actuation_window_s: float = 2.0,
        randomize_phase: bool = False,
        seed: int | None = None,
    ):
        self._fsm = IntersectionManager(
            num_envs=1,
            green_s=green_s,
            yellow_s=yellow_s,
            max_green_s=max_green_s,
            step_dt=step_dt,
            randomize_phase=randomize_phase,
            device="cpu",
            seed=seed,
        )
        self.actuation_ext_s = float(actuation_ext_s)
        self.actuation_window_s = float(actuation_window_s)
        self._intents: dict = {}

    # ------------------------- SchedulerCore-shaped API ------------------

    @property
    def active_intents(self) -> dict:
        return dict(self._intents)

    def register_intent(
        self,
        agent_id: str,
        intersection_id: str,
        turn_command,
        approach_group: int = 0,
        **_,
    ) -> float:
        """Register standing demand; returns the permission horizon (s).

        Horizon = time until this agent's movement permission next changes
        (green -> time left; stopped -> time until its green). Sync note:
        confirm against SchedulerCore's return semantics (ETA vs ack).
        """
        self._intents[agent_id] = {
            "intersection_id": intersection_id,
            "turn_command": turn_command,
            "group": int(approach_group),
        }
        return self._spat(int(approach_group))[1]

    def query_go_signal(self, agent_id: str, intersection_id=None, **_) -> float:
        info = self._intents.get(agent_id)
        if info is None:
            return 0.0  # unknown agent: fail safe
        return self._spat(info["group"])[0]

    def clear_agent(self, agent_id: str) -> None:
        self._intents.pop(agent_id, None)

    def tick(self) -> None:
        """Advance one control step, with actuated green extension.

        If any registered intent is on the currently-green group and that
        green is within actuation_window_s of ending, extend it by
        actuation_ext_s (IntersectionManager caps total green at
        max_green_s, so a saturated approach cannot starve the cross
        street).
        """
        if self._intents:
            demand_groups = {i["group"] for i in self._intents.values()}
            active = int(self._fsm.group[0].item())
            is_green = int(self._fsm.state[0].item()) == GREEN
            near_end = float(self._fsm.t_rem[0].item()) < self.actuation_window_s
            if active in demand_groups and is_green and near_end:
                self._fsm.extend_green(
                    torch.ones(1, dtype=torch.bool), self.actuation_ext_s
                )
        self._fsm.step()

    # ------------------------------ extras --------------------------------

    def permission_horizon(self, agent_id: str) -> float:
        """Seconds until the agent's permission next changes (V2I msg field)."""
        info = self._intents.get(agent_id)
        if info is None:
            return 0.0
        return self._spat(info["group"])[1]

    def phase(self, agent_id: str) -> int:
        """0 green, 1 yellow, 2 red (2 for unknown agents)."""
        info = self._intents.get(agent_id)
        if info is None:
            return 2
        return self._spat(info["group"])[2]

    # ------------------------------ internal ------------------------------

    def _spat(self, group: int):
        ag = torch.tensor([[group]], dtype=torch.long)
        go, ttc, ph = self._fsm.get_spat(ag)
        return float(go[0, 0]), float(ttc[0, 0]), int(ph[0, 0])
