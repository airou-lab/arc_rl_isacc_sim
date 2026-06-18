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

from .stop_line_detector import (
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
        stop_distance_threshold: float = 0.8,
        dwell_steps: int = 20,
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
        self.depart_clear_counter = torch.zeros(num_envs, dtype=torch.long, device=device)
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
        """Run detector + FSM step for every env. Returns the go_signal tensor.

        `images` is (N, H, W, 3) uint8 — the same tensor `get_image_uint8`
        produces. cv2 needs CPU numpy so we read each env's frame to CPU
        once per step. With num_envs=8 this is ~8x the per-image cv2 cost,
        which at 224x224 is sub-millisecond per env on this hardware.
        """
        if images is None:
            # Cameras disabled. Leave go_signal at whatever it currently
            # is — typically initialized to 1 (go).
            return self.go_signal

        imgs_cpu = images.detach().cpu().numpy()
        for i in range(self.num_envs):
            result = self.detector.detect(
                StopLineDetectionContext(image=imgs_cpu[i], active=True)
            )
            self._step_env(i, bool(result.detected), float(result.distance_m))
        return self.go_signal

    def _step_env(self, i: int, detected: bool, distance_m: float) -> None:
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
            # Stay in DEPART until detector has been "clear" for N steps.
            # Doesn't matter what bar we see along the way; the just-crossed
            # bar would otherwise immediately re-trigger STOP.
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


def get_go_signal_manager(
    num_envs: int, device: str
) -> GoSignalManager:
    """Lazy singleton; recreated if num_envs changes between calls."""
    global _GO_SIGNAL_MANAGER
    if (
        _GO_SIGNAL_MANAGER is None
        or _GO_SIGNAL_MANAGER.num_envs != num_envs
        or str(_GO_SIGNAL_MANAGER.device) != str(device)
    ):
        _GO_SIGNAL_MANAGER = GoSignalManager(num_envs=num_envs, device=device)
    return _GO_SIGNAL_MANAGER
