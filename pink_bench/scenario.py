#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

import pink
import pinocchio as pin

TrajectoryCallable = Callable[[pink.Task, float, Optional[dict]], None]


@dataclass
class Scenario:
    name: str
    duration: float
    robot_description: str
    n_joints: Optional[int] = None
    root_joint: Optional[pin.JointModel] = None
    initial_configuration: Dict[str, float] = field(default_factory=dict)
    trajectories: List[Tuple[pink.Task, TrajectoryCallable]] = field(
        default_factory=list
    )
