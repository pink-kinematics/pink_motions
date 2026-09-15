#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

from typing import Optional

import numpy as np
from loop_rate_limiters import RateLimiter

from .library import SCENARIOS
from .scene import Scene


def play_scenario(
    name: str,
    dt: float,
    qpsolver: str,
    visualize: bool = True,
    record: bool = False,
    plot_mpc_axis: Optional[str] = None,
):
    scenario = SCENARIOS[name]
    scene = Scene(scenario, visualize=visualize, record=record)
    if plot_mpc_axis is not None:
        scene.plot_mpc_axis(0 if plot_mpc_axis == "x" else 1)
    rate = RateLimiter(frequency=1.0 / dt, warn=False)
    for it_num, t in enumerate(np.arange(0.0, scenario.duration, dt)):
        scene.step(dt, solver=qpsolver)
        rate.sleep()
