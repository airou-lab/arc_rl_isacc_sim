"""Go-signal manager — per-env stop-bar state machine.

Runs the VisualStopLineDetector on each env's camera frame every step and
maintains a 3-state FSM per env:

    APPROACH:
        Normal driving. go_signal = 1.
        Transition: bar detected with distance < stop_distance_threshold
            → STOP, set go_signal = 0, reset dwell_counter.

    STOP:
        Dwelling at the bar. go_signal = 0 (action gate forces brake).
        Transition: dwell_counter reaches dwell_steps
            → DEPART, set go_signal = 1.

    DEPART:
        Driving through the intersection. go_signal = 1.
        We do NOT immediately go back to APPROACH because the just-crossed
        bar would re-trigger STOP. Wait until detector reports no bar for
        `depart_clear_steps` consecutive steps, then return to APPROACH.

The go_signal is published to `env.extras["go_signal"]` (shape (N,),
float32, 0.0/1.0). The telemetry observation reads it into slot 1; the
action term reads it to gate throttle (T3.3).

The detector itself is privileged from a *sim* standpoint (it sees the
sim render) but represents the deployment-realistic perception path:
exactly the same module will run on the real D435i camera stream. So
unlike lat_err/head_err (oracle geometric truth, PVP-masked from the
policy), go_signal IS shown to the policy — the deploy stack will
synthesize it the same way.
"""

from __future__ import annotations

import enum
from typing import Optional

import torch

try:
    from arcproLab.policy_stack.agent.stop_line_detector import (
        StopLineDetectionContext,
        StopLineDetectorConfig,
        VisualStopLineDetector,
    )
except ImportError:
    from policy_stack.agent.stop_line_detector import (
        StopLineDetectionContext,
        StopLineDetectorConfig,
        VisualStopLineDetector,
    )


class _State(enum.IntEnum):
    APPROACH = 0
    STOP = 1
    DEPART = 2


class GoSignalManager:
    """Per-env FSM driven by VisualStopLineDetector."""

    def __init__(
        self,
        num_envs: int,
        device: str,
        detector_config: Optional[StopLineDetectorConfig] = None,
        stop_distance_threshold: float = 0.45,
        dwell_steps: int = 40,
        depart_clear_steps: int = 15,
    ):
        self.num_envs = num_envs
        self.device = device
        self.detector = VisualStopLineDetector(detector_config)
        self.stop_distance_threshold = stop_distance_threshold
        self.dwell_steps = dwell_steps
        self.depart_clear_steps = depart_clear_steps

        self.state = torch.zeros(num_envs, dtype=torch.long, device=device)
        self.dwell_counter = torch.zeros(num_envs, dtype=torch.long, device=device)
        self.depart_clear_counter = torch.zeros(
            num_envs, dtype=torch.long, device=device
        )
        self.last_distance = torch.full(
            (num_envs,), float("nan"), dtype=torch.float32, device=device
        )
        self.last_detected = torch.zeros(num_envs, dtype=torch.bool, device=device)
        self.go_signal = torch.ones(num_envs, dtype=torch.float32, device=device)

    def reset(self, env_ids: torch.Tensor) -> None:
        """Zero out FSM state for the given env_ids (called on env reset)."""
        self.state[env_ids] = int(_State.APPROACH)
        self.dwell_counter[env_ids] = 0
        self.depart_clear_counter[env_ids] = 0
        self.last_distance[env_ids] = float("nan")
        self.last_detected[env_ids] = False
        self.go_signal[env_ids] = 1.0

    def update(self, images: torch.Tensor) -> torch.Tensor:
        """Run detector + vectorized FSM step for every env. Returns the go_signal tensor.

        `images` is (N, H, W, 3) uint8 — the same tensor `get_image_uint8`
        produces. cv2 needs CPU numpy so we read all frames to CPU once per
        step, run cv2 detection on CPU, and execute the FSM state update
        vectorized on `self.device` without per-env GPU-CPU `.item()` syncs.
        """
        if images is None:
            return self.go_signal

        imgs_cpu = images.detach().cpu().numpy()
        det_list = []
        dist_list = []
        for i in range(self.num_envs):
            result = self.detector.detect(
                StopLineDetectionContext(image=imgs_cpu[i], active=True)
            )
            det_list.append(bool(result.detected))
            dist_list.append(float(result.distance_m))

        detected_t = torch.tensor(det_list, dtype=torch.bool, device=self.device)
        dist_t = torch.tensor(dist_list, dtype=torch.float32, device=self.device)

        is_approach = self.state == int(_State.APPROACH)
        is_stop = self.state == int(_State.STOP)
        is_depart = self.state == int(_State.DEPART)

        # APPROACH transitions
        trigger_stop = is_approach & detected_t & (dist_t < self.stop_distance_threshold)

        # STOP transitions
        next_dwell = self.dwell_counter + 1
        finish_stop = is_stop & (next_dwell >= self.dwell_steps)
        stay_stop = is_stop & (~finish_stop)

        # DEPART transitions
        next_clear = torch.where(
            detected_t,
            torch.zeros_like(self.depart_clear_counter),
            self.depart_clear_counter + 1,
        )
        finish_depart = is_depart & (~detected_t) & (next_clear >= self.depart_clear_steps)

        # Apply state updates atomically
        new_state = self.state.clone()
        new_state = torch.where(
            trigger_stop,
            torch.full_like(new_state, int(_State.STOP)),
            new_state,
        )
        new_state = torch.where(
            finish_stop,
            torch.full_like(new_state, int(_State.DEPART)),
            new_state,
        )
        new_state = torch.where(
            finish_depart,
            torch.full_like(new_state, int(_State.APPROACH)),
            new_state,
        )
        self.state = new_state

        self.dwell_counter = torch.where(
            trigger_stop,
            torch.zeros_like(self.dwell_counter),
            torch.where(is_stop, next_dwell, self.dwell_counter),
        )
        self.depart_clear_counter = torch.where(
            finish_stop,
            torch.zeros_like(self.depart_clear_counter),
            torch.where(
                is_depart,
                torch.where(
                    finish_depart,
                    torch.zeros_like(next_clear),
                    next_clear,
                ),
                self.depart_clear_counter,
            ),
        )

        is_red = trigger_stop | stay_stop
        self.go_signal = torch.where(
            is_red,
            torch.zeros_like(self.go_signal),
            torch.ones_like(self.go_signal),
        )

        self.last_distance = dist_t
        self.last_detected = detected_t
        return self.go_signal

    def _step_env(self, i: int, detected: bool, distance_m: float) -> None:
        """Single-env transition helper retained for unit-test compatibility."""
        state = int(self.state[i].item())

        if state == _State.APPROACH:
            if detected and distance_m < self.stop_distance_threshold:
                self.state[i] = int(_State.STOP)
                self.dwell_counter[i] = 0
                self.go_signal[i] = 0.0
            else:
                self.go_signal[i] = 1.0

        elif state == _State.STOP:
            self.dwell_counter[i] += 1
            if int(self.dwell_counter[i].item()) >= self.dwell_steps:
                self.state[i] = int(_State.DEPART)
                self.depart_clear_counter[i] = 0
                self.go_signal[i] = 1.0
            else:
                self.go_signal[i] = 0.0

        elif state == _State.DEPART:
            if not detected:
                self.depart_clear_counter[i] += 1
                if int(self.depart_clear_counter[i].item()) >= self.depart_clear_steps:
                    self.state[i] = int(_State.APPROACH)
                    self.depart_clear_counter[i] = 0
            else:
                self.depart_clear_counter[i] = 0
            self.go_signal[i] = 1.0

        self.last_distance[i] = distance_m
        self.last_detected[i] = detected


# Singleton accessor — same pattern as TrackManager.
_GO_SIGNAL_MANAGER: Optional[GoSignalManager] = None


def get_go_signal_manager(num_envs: int, device: str) -> GoSignalManager:
    """Lazy singleton; recreated if num_envs changes between calls."""
    global _GO_SIGNAL_MANAGER
    if (
        _GO_SIGNAL_MANAGER is None
        or _GO_SIGNAL_MANAGER.num_envs != num_envs
        or str(_GO_SIGNAL_MANAGER.device) != str(device)
    ):
        _GO_SIGNAL_MANAGER = GoSignalManager(num_envs=num_envs, device=device)
    return _GO_SIGNAL_MANAGER


import sys as _sys  # noqa: E402

_sys.modules.setdefault("mdp.go_signal_manager", _sys.modules[__name__])
_sys.modules.setdefault("arcproLab.mdp.go_signal_manager", _sys.modules[__name__])

