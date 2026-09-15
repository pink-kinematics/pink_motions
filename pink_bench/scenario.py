# SPDX-License-Identifier: Apache-2.0

"""Scenario dataclass."""

from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

import pink
import pinocchio as pin

TrajectoryCallable = Callable[[pink.Task, float, Optional[dict]], None]


@dataclass
class Scenario:
    """Scenario dataclass."""

    name: str
    duration: float
    robot_description: str
    n_joints: Optional[int] = None
    root_joint: Optional[pin.JointModel] = None
    initial_configuration: Dict[str, float] = field(default_factory=dict)
    trajectories: List[Tuple[pink.Task, TrajectoryCallable]] = field(
        default_factory=list
    )

    # Optional uniform velocity limit, in [rad] / [s], applied to every joint.
    # Some robots have only continuous joints and hence no configuration or
    # velocity limit in their model (e.g. edo and poppy_ergo_jr), so their IK
    # problem would be an unconstrained QP. The custom velocity limit bounds
    # it. The default is ``None`` for robots that already have limits.
    velocity_limit: Optional[float] = None
