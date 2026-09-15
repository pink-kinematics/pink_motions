#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
#
# /// script
# requires-python = ">=3.9"
# dependencies = [
#   "loop-rate-limiters>=0.1.0",
#   "numpy",
#   "pink_bench",
#   "qpsolvers>=4.3.1",
# ]
# ///

"""Create a scene and step it problem by problem."""

import argparse

import numpy as np
import qpsolvers
from loop_rate_limiters import RateLimiter
from pink import build_ik

import pink_bench

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "scenario",
        help="scenario to run from the pink_bench library",
        choices=list([name for name in pink_bench.SCENARIOS.keys()]),
    )
    parser.add_argument(
        "--dt",
        help="Timestep in seconds between differential IK problems",
        type=float,
        default=0.005,
    )
    parser.add_argument(
        "--plot-mpc-axis",
        help="plot to debug LIP model predictive control",
        choices=("x", "y", None),
        default=None,
    )
    parser.add_argument(
        "--qpsolver",
        help="solver for the QP-based approach",
        default="clarabel",
        choices=qpsolvers.available_solvers,
    )
    args = parser.parse_args()
    scenario = pink_bench.SCENARIOS[args.scenario]
    scene = pink_bench.Scene(scenario, visualize=True)
    if args.plot_mpc_axis is not None:
        scene.plot_mpc_axis(0 if args.plot_mpc_axis == "x" else 1)
    rate = RateLimiter(frequency=1.0 / args.dt, warn=False)
    for it_num, t in enumerate(np.arange(0.0, scenario.duration, args.dt)):
        scene.step_targets(args.dt)
        qp = build_ik(scene.configuration, scene.tasks, args.dt)
        result = qpsolvers.solve_problem(qp, solver=args.qpsolver)
        Delta_q = result.x
        if Delta_q is None:
            raise ValueError("Could not solve differential IK problem")
        velocity = Delta_q / args.dt
        scene.step_velocity(velocity, args.dt)
        rate.sleep()
