"""Curriculum & Sentinel Configuration Manager for ARCPro RL."""

import datetime
import json
import os
from typing import Any, Dict, List, Tuple

try:
    from mdp.track_topology import (
        DEFAULT_LOOKAHEAD_DISTANCE,
        DEFAULT_SECTOR_WEIGHTS,
        SECTOR_DEFS,
        SECTOR_DESCRIPTIONS,
    )
except ImportError:
    from arcproLab.mdp.track_topology import (
        DEFAULT_LOOKAHEAD_DISTANCE,
        DEFAULT_SECTOR_WEIGHTS,
        SECTOR_DEFS,
        SECTOR_DESCRIPTIONS,
    )

_CONFIG_PATH = os.path.join(os.path.dirname(__file__), "curriculum_sectors.json")
_LAST_MTIME = 0.0
_DEAD_KEYS = ("speed_tracking_weight", "target_corner_speed")

_CACHED_CONFIG: Dict[str, Any] = {
    "sector_weights": list(DEFAULT_SECTOR_WEIGHTS),
    "lookahead_distance": DEFAULT_LOOKAHEAD_DISTANCE,
    "sectors": dict(SECTOR_DESCRIPTIONS),
}
_CACHED_THRESHOLDS: List[float] = [0.25, 0.50, 0.70, 0.85]


def _recompute_thresholds(weights: List[float]) -> List[float]:
    if len(weights) == 5:
        total = float(sum(weights))
        if total > 0:
            w = [float(x) / total for x in weights]
            return [
                w[0],
                w[0] + w[1],
                w[0] + w[1] + w[2],
                w[0] + w[1] + w[2] + w[3],
            ]
    return [0.25, 0.50, 0.70, 0.85]


def get_curriculum_config() -> Dict[str, Any]:
    """Load curriculum config with mtime-based auto-reload."""
    global _LAST_MTIME, _CACHED_CONFIG, _CACHED_THRESHOLDS
    if os.path.exists(_CONFIG_PATH):
        try:
            mtime = os.path.getmtime(_CONFIG_PATH)
            if mtime != _LAST_MTIME:
                with open(_CONFIG_PATH, "r") as f:
                    data = json.load(f)
                for k in _DEAD_KEYS:
                    data.pop(k, None)
                _CACHED_CONFIG.update(data)
                for k in _DEAD_KEYS:
                    _CACHED_CONFIG.pop(k, None)
                weights = _CACHED_CONFIG.get(
                    "sector_weights", list(DEFAULT_SECTOR_WEIGHTS)
                )
                _CACHED_THRESHOLDS = _recompute_thresholds(weights)
                _LAST_MTIME = mtime
        except Exception:
            pass
    return _CACHED_CONFIG


def get_sector_thresholds() -> Tuple[float, float, float, float]:
    """Get the 4 cumulative probability thresholds for 5 sectors."""
    get_curriculum_config()
    return (
        _CACHED_THRESHOLDS[0],
        _CACHED_THRESHOLDS[1],
        _CACHED_THRESHOLDS[2],
        _CACHED_THRESHOLDS[3],
    )


def get_lookahead_distance() -> float:
    """Get dynamic lookahead distance for route curvature sampling."""
    cfg = get_curriculum_config()
    return float(cfg.get("lookahead_distance", DEFAULT_LOOKAHEAD_DISTANCE))


def update_curriculum_config(updates: Dict[str, Any]) -> Dict[str, Any]:
    """Atomically update the curriculum JSON configuration file."""
    cfg = get_curriculum_config().copy()
    clean_updates = {k: v for k, v in updates.items() if k not in _DEAD_KEYS}
    cfg.update(clean_updates)
    for k in _DEAD_KEYS:
        cfg.pop(k, None)

    cfg["updated_at"] = datetime.datetime.utcnow().isoformat() + "Z"

    tmp_path = _CONFIG_PATH + ".tmp"
    with open(tmp_path, "w") as f:
        json.dump(cfg, f, indent=2)
    os.replace(tmp_path, _CONFIG_PATH)
    get_curriculum_config()
    return cfg


import sys as _sys  # noqa: E402

_sys.modules.setdefault("mdp.curriculum_config", _sys.modules[__name__])
_sys.modules.setdefault("arcproLab.mdp.curriculum_config", _sys.modules[__name__])

