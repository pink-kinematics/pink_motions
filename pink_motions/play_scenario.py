# SPDX-License-Identifier: Apache-2.0

"""Play a scenario of the library, in real time."""

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
    """Unroll a scenario of the library, pacing it to real time.

    Args:
        name: Name of the scenario in the library.
        dt: Timestep between differential IK problems, in seconds. The
            scenario is played back at that rate.
        qpsolver: Backend QP solver.
        visualize: If true (default), display the robot in MeshCat.
        record: If true, save a picture of each frame to ``videos/``.
        plot_mpc_axis: Axis, ``"x"`` or ``"y"``, whose model predictive
            control plan to plot live. Defaults to no plot.
    """
    scenario = SCENARIOS[name]
    scene = Scene(scenario, visualize=visualize, record=record)
    if plot_mpc_axis is not None:
        scene.plot_mpc_axis(0 if plot_mpc_axis == "x" else 1)
    rate = RateLimiter(frequency=1.0 / dt, warn=False)
    for it_num, t in enumerate(np.arange(0.0, scenario.duration, dt)):
        scene.step(dt, solver=qpsolver)
        rate.sleep()
