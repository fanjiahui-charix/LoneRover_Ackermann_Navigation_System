#!/usr/bin/env python3
"""Stable CLI location for the Tube-v2 candidate generator."""

from __future__ import annotations

import sys
from pathlib import Path

TOOLS = Path(__file__).resolve().parents[1]
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))

from generate_channel_tubes_v2 import main  # noqa: E402


if __name__ == "__main__":
    raise SystemExit(main())
