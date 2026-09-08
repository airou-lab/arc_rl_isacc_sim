"""Curriculum & Sentinel Configuration Manager for ARCPro RL."""

import json
import os
from typing import Any, Dict, List, Tuple

_CONFIG_PATH = os.path.join(os.path.dirname(__file__), "curriculum_sectors.json")
_LAST_MTIME = 0.0
_CACHED_CONFIG: Dict[str, Any] = {
    "sector_weights": [0.20, 0.20, 0.20, 0.20, 0.20],
    "lookahead_distance": 1.2,
    "speed_tracking_weight": 60.0,
    "target_corner_speed": 0.35,
    "sectors": {
        "1": "Start Line (11m straight)",
        "2": "Corner 1 Runway Approach (2.5m runway)",
        "3": "Leg 2 Post-C1 Exit (8m runway)",
        "4": "Leg 2 Intersection Approach (4.15m runway)",
        "5": "North Straight Return Corridor (10m runway)"
    }
}
_CACHED_THRESHOLDS: List[float] = [0.20, 0.40, 0.60, 0.80]


def get_curriculum_config() -> Dict[str, Any]:
    """Load curriculum config with mtime-based auto-reload."""
    global _LAST_MTIME, _CACHED_CONFIG, _CACHED_THRESHOLDS
    if os.path.exists(_CONFIG_PATH):
        try:
            mtime = os.path.getmtime(_CONFIG_PATH)
            if mtime != _LAST_MTIME:
                with open(_CONFIG_PATH, "r") as f:
                    data = json.load(f)
                _CACHED_CONFIG.update(data)
                weights = _CACHED_CONFIG.get("sector_weights", [0.20, 0.20, 0.20, 0.20, 0.20])
                if len(weights) == 5:
                    total = float(sum(weights))
                    if total > 0:
                        w = [float(x) / total for x in weights]
                        _CACHED_THRESHOLDS = [
                            w[0],
                            w[0] + w[1],
                            w[0] + w[1] + w[2],
                            w[0] + w[1] + w[2] + w[3],
                        ]
                _LAST_MTIME = mtime
        except Exception:
            pass
    return _CACHED_CONFIG


def get_sector_thresholds() -> Tuple[float, float, float, float]:
    """Get the 4 cumulative probability thresholds for 5 sectors."""
    get_curriculum_config()
    return (_CACHED_THRESHOLDS[0], _CACHED_THRESHOLDS[1], _CACHED_THRESHOLDS[2], _CACHED_THRESHOLDS[3])


def get_lookahead_distance() -> float:
    """Get dynamic lookahead distance for route curvature sampling."""
    cfg = get_curriculum_config()
    return float(cfg.get("lookahead_distance", 1.2))


def update_curriculum_config(updates: Dict[str, Any]) -> Dict[str, Any]:
    """Atomically update the curriculum JSON configuration file."""
    cfg = get_curriculum_config().copy()
    cfg.update(updates)
    import datetime
    cfg["updated_at"] = datetime.datetime.utcnow().isoformat() + "Z"
    
    tmp_path = _CONFIG_PATH + ".tmp"
    with open(tmp_path, "w") as f:
        json.dump(cfg, f, indent=2)
    os.replace(tmp_path, _CONFIG_PATH)
    # Trigger refresh
    get_curriculum_config()
    return cfg
