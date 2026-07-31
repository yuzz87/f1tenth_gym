"""Locate the repository core used by the ROS 2 wrapper package."""

import os
from pathlib import Path
import sys


def find_repository_root():
    configured = os.environ.get("F1TENTH_GYM_ROOT")
    candidates = [Path(configured).expanduser()] if configured else []
    candidates.extend(Path(__file__).resolve().parents)
    for candidate in candidates:
        marker = candidate / "experiments/real_vehicle/config/real_vehicle.yaml"
        if marker.is_file():
            return candidate
    raise RuntimeError(
        "cannot locate f1tenth_gym; set F1TENTH_GYM_ROOT to the repository root"
    )


def enable_repository_imports():
    root = find_repository_root()
    root_text = str(root)
    if root_text not in sys.path:
        sys.path.insert(0, root_text)
    return root
