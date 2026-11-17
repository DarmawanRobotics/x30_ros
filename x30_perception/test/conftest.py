"""The tests share livox_pack and depth_scene from this directory."""

from __future__ import annotations

from pathlib import Path
import sys

HERE = str(Path(__file__).resolve().parent)
if HERE not in sys.path:
    sys.path.insert(0, HERE)
