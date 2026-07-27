"""Stop Line Detector — visual variant.

Ported (subset) from arc_rl_isacc_policy/agent/stop_line_detector.py
(commit e200b27 'feat(policy): swap NatureCNN for ResNet-18 at 224x224')
for use inside the ARCPro sim training loop.

This file intentionally ports ONLY the visual detector. The geometric
variant in the policy repo depends on IntersectionLayout +
distance_to_stop_line_world from agent/intersection_geometry.py, which
we have not yet ported. Geometric is a privileged training bootstrap;
visual is the deployment-realistic path. T3.1's goal is to validate the
visual path on Isaac-rendered frames, so visual is the right starting
point.

Conventions match policy repo's lane_detector.py:
    - dataclass return type with confidence in [0, 1]
    - class with __init__(config) and detect(ctx) -> Result
    - cv2 for the visual variant (already a sim repo dependency)
    - stateless per-frame (no tracker)

Pixel-to-ground projection (visual variant) uses pinhole + level-ground:
    d = camera_height / tan(pitch_angle)
where pitch_angle is derived from pixel position + intrinsics.

Camera intrinsics align with ARCProSceneCfg (horizontal_aperture=2.65,
focal_length=1.93, 224x224 image). camera_height_m=0.20 matches the
chassis+mount sum at 1.0x metric scale.
"""

from __future__ import annotations

import math
import logging
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None

logger = logging.getLogger(__name__)


# Detection output

@dataclass
class StopLineDetection:
    """Result of one stop-line detection attempt.

    A null / no-detection result has detected=False and the numeric
    fields zeroed for predictability.
    """
    detected: bool = False
    distance_m: float = 0.0
    confidence: float = 0.0
    image_row: Optional[int] = None
    source: str = "none"
    # Visualization hints (populated only when detected=True).
    band_top_row: Optional[int] = None
    band_bottom_row: Optional[int] = None
    white_threshold: Optional[int] = None


@dataclass
class StopLineDetectionContext:
    """Per-frame inputs to a StopLineDetector."""
    image: Optional[np.ndarray] = None
    agent_xy: Optional[Tuple[float, float]] = None
    intersection_center: Optional[Tuple[float, float]] = None
    approach_heading_rad: Optional[float] = None
    active: bool = True   # default True for sim; in policy repo Worker gates this


# Config

@dataclass
class StopLineDetectorConfig:
    """Configuration for VisualStopLineDetector. Defaults align with
    ARCProSceneCfg camera at 1.0x metric scale.
    """
    img_width: int = 224
    img_height: int = 224
    horizontal_aperture: float = 2.65
    focal_length: float = 1.93
    camera_height_m: float = 0.20
    camera_pitch_rad: float = 0.0

    roi_top_ratio: float = 0.5
    white_threshold: int = 200
    # Pixel thresholds inherited from the policy repo's 224x224 retune.
    min_line_width_px: int = 20
    max_line_thickness_px: int = 20
    min_fraction: float = 0.35
    min_confidence: float = 0.3

    # Distance-aware detection.
    # A real stop bar of physical width `expected_bar_width_m` projects to
    # fewer image pixels the farther it is. At distance D ahead of the camera
    # the bar occupies (bar_width / (2 * D * tan(HFOV/2))) of the image width.
    # We accept a row whose contiguous bright run is at least
    # `distance_aware_margin * expected_pixels(D(row))`, OR at least
    # `min_fraction * img_width` — whichever is smaller (so the strict
    # min_fraction acts as a SAFETY CAP for very close detections rather
    # than a global floor that rejects all distant bars).
    expected_bar_width_m: float = 0.6
    distance_aware_margin: float = 0.65
    # Hard distance cap: don't trust detections beyond this range. Beyond ~2.5m
    # the expected pixel count drops below min_line_width_px=20 anyway, and the
    # bar's contrast against road texture is too low for reliable detection.
    max_detection_distance_m: float = 2.5

    @property
    def vertical_aperture(self) -> float:
        return self.horizontal_aperture * self.img_height / self.img_width

    @property
    def tan_half_hfov(self) -> float:
        """tan of half the horizontal FOV. Used for distance-to-pixel mapping."""
        return (self.horizontal_aperture / 2.0) / self.focal_length


# Base

class StopLineDetectorBase:
    source: str = "none"

    def detect(self, ctx: StopLineDetectionContext) -> StopLineDetection:
        raise NotImplementedError


# Visual

def _longest_run_per_row(binary: np.ndarray) -> np.ndarray:
    """For each row of a 2-D bool array, length of the longest contiguous
    True run. Returns int array of shape (binary.shape[0],).
    """
    h, w = binary.shape
    out = np.zeros(h, dtype=np.int32)
    for r in range(h):
        row = binary[r]
        if not row.any():
            continue
        padded = np.concatenate(([0], row.astype(np.int8), [0]))
        diff = np.diff(padded)
        starts = np.where(diff == 1)[0]
        ends = np.where(diff == -1)[0]
        out[r] = int((ends - starts).max())
    return out


class VisualStopLineDetector(StopLineDetectorBase):
    """Classical CV stop-line detector.

    Pipeline:
        1. If not active, return null.
        2. Extract lower-half ROI (mask out above-horizon region).
        3. Convert to grayscale, threshold for bright pixels.
        4. Compute row-wise white-pixel counts.
        5. Find peak row with strongest horizontal white run.
        6. Reject if peak fraction below threshold (not a real line).
        7. Reject if bright region spans too many rows (not a stripe).
        8. Project peak row to ahead-of-camera ground distance.
        9. Confidence = peak_fraction (clamped).
    """

    source = "visual"

    def __init__(self, config: Optional[StopLineDetectorConfig] = None):
        if cv2 is None:
            raise ImportError(
                "VisualStopLineDetector requires opencv-python (cv2)."
            )
        self.config = config or StopLineDetectorConfig()
        self._roi_top = int(self.config.img_height * self.config.roi_top_ratio)

    def detect(self, ctx: StopLineDetectionContext) -> StopLineDetection:
        if not ctx.active or ctx.image is None:
            return StopLineDetection(source=self.source)

        image = ctx.image
        h, w = self.config.img_height, self.config.img_width
        if image.shape[0] != h or image.shape[1] != w:
            logger.debug(
                "VisualStopLineDetector: image shape %s != expected (%d, %d)",
                image.shape, h, w,
            )
            return StopLineDetection(source=self.source)

        # Grayscale (accepts RGB or single-channel uint8)
        if image.ndim == 3 and image.shape[2] >= 3:
            gray = cv2.cvtColor(image[:, :, :3], cv2.COLOR_RGB2GRAY)
        else:
            gray = image

        # Mask upper region (above horizon)
        mask = gray.copy()
        mask[: self._roi_top, :] = 0

        # Threshold for white paint
        _, white = cv2.threshold(
            mask, self.config.white_threshold, 255, cv2.THRESH_BINARY
        )

        # Horizontal whiteness per row — LONGEST CONTIGUOUS run, not total
        # count. Two narrow lane curbs (~28% left + ~10% right) summing to
        # 38% of the width would otherwise pass min_fraction=0.35 and be
        # mistaken for a real stop bar, which is exactly the false-positive
        # mode we saw in the post-spawn capture sequence.
        row_counts = _longest_run_per_row(white > 0)
        if row_counts.max() == 0:
            return StopLineDetection(source=self.source)

        # Distance-aware per-row qualification. Build a per-row threshold from
        # the bar's expected pixel width at the distance projected from that
        # row, then pick the row with the largest contiguous run AMONG those
        # that clear their per-row threshold. Picking the global argmax row
        # blindly would miss a far bar when a near-range noisy row had more
        # pixels but failed its (higher) close-range threshold.
        per_row_thresh = self._per_row_thresholds(h, w)
        qualifying = row_counts >= per_row_thresh
        if not qualifying.any():
            return StopLineDetection(source=self.source)

        # argmax over masked array; non-qualifying rows score -1.
        masked = np.where(qualifying, row_counts, -1)
        peak_row = int(np.argmax(masked))
        peak_count = int(row_counts[peak_row])

        # Line-width gate (noise floor; mostly redundant with per_row_thresh).
        if peak_count < self.config.min_line_width_px:
            return StopLineDetection(source=self.source)

        # Thickness gate: reject large bright regions
        neighborhood_threshold = max(
            int(peak_count * 0.6), self.config.min_line_width_px
        )
        above_thresh = row_counts >= neighborhood_threshold
        run_len = 1
        band_top = peak_row
        band_bottom = peak_row
        r = peak_row - 1
        while r >= 0 and above_thresh[r]:
            run_len += 1
            band_top = r
            r -= 1
        r = peak_row + 1
        while r < h and above_thresh[r]:
            run_len += 1
            band_bottom = r
            r += 1
        if run_len > self.config.max_line_thickness_px:
            return StopLineDetection(source=self.source)

        distance = self._row_to_distance(peak_row)
        if distance is None:
            return StopLineDetection(source=self.source)
        if distance > self.config.max_detection_distance_m:
            return StopLineDetection(source=self.source)

        # Per-row threshold already qualified the peak; compute confidence
        # from how strongly the row beats its expected pixel count. A bar
        # that exceeds its expected width (closer / wider than predicted)
        # saturates to 1.0.
        expected_pixels = self._expected_bar_pixels(distance)
        confidence = min(peak_count / max(expected_pixels, 1.0), 1.0)
        if confidence < self.config.min_confidence:
            return StopLineDetection(source=self.source)

        return StopLineDetection(
            detected=True,
            distance_m=float(distance),
            confidence=float(confidence),
            image_row=peak_row,
            source=self.source,
            band_top_row=int(band_top),
            band_bottom_row=int(band_bottom),
            white_threshold=int(self.config.white_threshold),
        )

    def _row_to_distance(self, row: int) -> Optional[float]:
        cfg = self.config
        H = cfg.img_height
        pixel_pitch = cfg.vertical_aperture / H
        y_img = (row - H / 2.0) * pixel_pitch
        pitch_angle = cfg.camera_pitch_rad + math.atan2(y_img, cfg.focal_length)
        if pitch_angle <= 1e-4:
            return None
        return cfg.camera_height_m / math.tan(pitch_angle)

    def _per_row_thresholds(self, h: int, w: int) -> np.ndarray:
        """Vectorized per-row minimum-contiguous-run threshold.

        For each row r in [0, h):
            - if r is above the ROI top, or above the horizon, or projects
              to a distance beyond max_detection_distance_m → +inf
              (effectively disqualifies the row)
            - otherwise:
                expected = expected bar pixel-width at distance(r)
                thresh   = max(
                    min_line_width_px,
                    min(distance_aware_margin * expected, min_fraction * w),
                )

        The min() in the middle is what makes the gate distance-aware:
            - Close (large expected) → min_fraction*w wins → strict 35% gate
            - Far  (small expected) → distance_aware_margin*expected wins
              → relaxed, scales with the bar's true pixel projection
        """
        cfg = self.config
        H = cfg.img_height
        pixel_pitch = cfg.vertical_aperture / H
        focal = cfg.focal_length

        row_idx = np.arange(h, dtype=np.float32)
        y_img = (row_idx - H / 2.0) * pixel_pitch
        pitch = cfg.camera_pitch_rad + np.arctan2(y_img, focal)
        # Above-horizon rows (pitch ~ 0): infinite distance, will be masked.
        below_horizon = pitch > 1e-4

        distances = np.full(h, np.inf, dtype=np.float32)
        distances[below_horizon] = cfg.camera_height_m / np.tan(pitch[below_horizon])

        # FOV width at each row's projected distance.
        fov_w = 2.0 * distances * cfg.tan_half_hfov
        # Expected pixel width: clip at img width.
        with np.errstate(divide="ignore", invalid="ignore"):
            expected = np.where(
                fov_w > 1e-6,
                cfg.expected_bar_width_m * w / fov_w,
                w,
            )
        expected = np.clip(expected, 0, w)

        distance_thresh = cfg.distance_aware_margin * expected
        abs_min_thresh = cfg.min_fraction * w
        thresh = np.maximum(
            float(cfg.min_line_width_px),
            np.minimum(distance_thresh, abs_min_thresh),
        )

        # Disqualify rows above the ROI top, above the horizon, or beyond
        # the max detection range.
        bad = (
            (row_idx < self._roi_top)
            | (~below_horizon)
            | (distances > cfg.max_detection_distance_m)
        )
        thresh[bad] = np.inf
        return thresh

    def _expected_bar_pixels(self, distance_m: float) -> float:
        """Expected pixel-width of a `expected_bar_width_m`-meter bar at
        distance `distance_m` ahead of the camera.

        At distance D the camera's horizontal field of view spans
        2 * D * tan(HFOV/2) meters of ground. A bar of width b therefore
        fills b / (2 * D * tan(HFOV/2)) of the image width.

        Returns the pixel-width as a float (not clamped to int) for use as
        a per-row qualification threshold. Returns img_width for D <= 0
        (defensive: the bar would fill the entire frame).
        """
        cfg = self.config
        if distance_m <= 1e-3:
            return float(cfg.img_width)
        fov_width = 2.0 * distance_m * cfg.tan_half_hfov
        if fov_width <= 1e-6:
            return float(cfg.img_width)
        return min(
            float(cfg.img_width),
            cfg.expected_bar_width_m * cfg.img_width / fov_width,
        )


# Overlay helper

def visualize_stop_line_detection(
    image: np.ndarray,
    result: StopLineDetection,
    mask_alpha: float = 0.55,
) -> np.ndarray:
    """Annotate `image` with the detection. Requires cv2.

    Paints a translucent green mask over the actual bright pixels that
    drove the detection — i.e. every pixel >= white_threshold in the
    vertical band [band_top_row, band_bottom_row]. This makes it visually
    obvious whether the detector locked onto a real stop bar or onto
    curbs / lane edges / texture artifacts.

    Returns a new annotated image (RGB) regardless of detection outcome.
    """
    if cv2 is None:
        return image
    vis = image.copy()
    h, w = vis.shape[:2]
    color = (0, 255, 0)

    if (
        result.detected
        and result.image_row is not None
        and result.band_top_row is not None
        and result.band_bottom_row is not None
        and result.white_threshold is not None
    ):
        top = max(0, int(result.band_top_row))
        bot = min(h - 1, int(result.band_bottom_row))
        # Re-derive the threshold mask in the band.
        if image.ndim == 3 and image.shape[2] >= 3:
            gray = cv2.cvtColor(image[:, :, :3], cv2.COLOR_RGB2GRAY)
        else:
            gray = image
        band_gray = gray[top:bot + 1, :]
        bright = band_gray >= int(result.white_threshold)
        band_vis = vis[top:bot + 1, :]
        green = np.array(color, dtype=np.float32)
        # Alpha-blend: bright pixels get tinted green, others untouched.
        if bright.any():
            blended = (
                mask_alpha * green
                + (1.0 - mask_alpha) * band_vis[bright].astype(np.float32)
            ).clip(0, 255).astype(np.uint8)
            band_vis[bright] = blended
        # Thin outline at the peak row, plus distance label.
        cv2.line(vis, (0, result.image_row), (w - 1, result.image_row), color, 1)
        cv2.putText(
            vis, f"{result.distance_m:.2f}m",
            (5, max(12, result.image_row - 3)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1,
        )

    cv2.putText(
        vis, f"src={result.source} conf={result.confidence:.2f} det={result.detected}",
        (5, h - 6),
        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1,
    )
    return vis
