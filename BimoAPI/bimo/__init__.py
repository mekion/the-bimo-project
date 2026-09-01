# Copyright (c) 2025-2026, Mekion
# SPDX-License-Identifier: Apache-2.0
"""Public API for the Bimo Robotics Kit."""

from .bimo import Bimo
from .routines import BimoRoutines
from .cpg import BimoCPG

__all__ = ["Bimo", "BimoRoutines", "BimoCPG"]
__version__ = "1.1.0"
