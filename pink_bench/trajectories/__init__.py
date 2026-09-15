# SPDX-License-Identifier: Apache-2.0

"""Task-target trajectories available to bench scenarios."""

from .back_and_forth_trajectory import BackAndForthTrajectory
from .lipm_walking_trajectory import LIPMWalkingTrajectory
from .swing_foot_trajectory import SwingFootTrajectory

__all__ = [
    "BackAndForthTrajectory",
    "LIPMWalkingTrajectory",
    "SwingFootTrajectory",
]
