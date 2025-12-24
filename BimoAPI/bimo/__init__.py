# Copyright (c) 2025, Mekion
# SPDX-License-Identifier: Apache-2.0
"""Public API for the Bimo Robotics Kit."""

from .bimo import Bimo
from .routines import BimoRoutines

__all__ = ["Bimo", "BimoRoutines"]
__version__ = "0.1.0"
