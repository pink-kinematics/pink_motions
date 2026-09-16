# SPDX-License-Identifier: Apache-2.0
#
# /// script
# requires-python = ">=3.11"
# dependencies = ["clarabel", "meshcat-shapes", "pink_motions"]
# ///

"""Run a given scenario from the library."""

import argparse

import qpsolvers

import pink_motions

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--robot",
        required=True,
        help="scenario to run from the pink motions library",
        choices=list([name for name in pink_motions.SCENARIOS.keys()]),
    )
    parser.add_argument(
        "--qpsolver",
        help="solver for the QP-based approach",
        default="clarabel",
        choices=qpsolvers.available_solvers,
    )
    args = parser.parse_args()
    pink_motions.play_scenario(
        name=args.robot,
        dt=0.005,
        qpsolver=args.qpsolver,
    )
