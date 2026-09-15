#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
#
# /// script
# requires-python = ">=3.9"
# dependencies = [
#   "pink_bench",
#   "qpsolvers>=4.3.1",
# ]
# ///

import argparse
import glob
import os
import subprocess
from pathlib import Path
from typing import List

import qpsolvers

import pink_bench


def parse_command_line_arguments() -> argparse.Namespace:
    """Parse command-line arguments.

    Returns:
        Namespace resulting from parsing command-line arguments.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--qpsolver",
        help="solver for the QP-based approach",
        default="clarabel",
        choices=qpsolvers.available_solvers,
    )
    parser.add_argument(
        "--scenario",
        help="record video only for a single scenario",
        choices=list([name for name in pink_bench.SCENARIOS.keys()]),
    )
    parser.add_argument(
        "--dt",
        help="Timestep in seconds between differential IK problems",
        type=float,
        default=0.005,
    )
    return parser.parse_args()


def save_video(name: str, frequency: int) -> None:
    video_output = f"videos/{name}.mp4"
    print(f"Saving video to {video_output}...")
    subprocess.run(
        [
            "ffmpeg",
            "-r",
            f"{frequency}",
            "-f",
            "image2",
            "-s",
            "1920x1080",
            "-i",
            f"videos/{name}_%04d.png",
            "-vcodec",
            "libx264",
            "-crf",
            "20",
            "-pix_fmt",
            "yuv420p",
            "-y",
            video_output,
        ]
    )
    print(f"Saved video to {video_output}")

    save_dir = f"videos/src-{name}"
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    for img in glob.glob(f"videos/{name}_*.png"):
        subprocess.run(["mv", img, save_dir])
    print(f"Backed up images to {save_dir}")


if __name__ == "__main__":
    args = parse_command_line_arguments()
    scenarios: List[str] = (
        [args.scenario]
        if args.scenario is not None
        else list(pink_bench.SCENARIOS.keys())
    )
    data_dir = Path(__file__).resolve().parent.parent / "data"
    for scenario_name in scenarios:
        if os.path.exists(f"videos/{scenario_name}.mp4"):
            print(f"Skipping {scenario_name} as video exists...")
            continue
        print(f'Recording video for scenario "{scenario_name}"...')
        scenario = pink_bench.SCENARIOS[scenario_name]
        pink_bench.play_scenario(
            name=scenario_name,
            dt=args.dt,
            qpsolver=args.qpsolver,
            visualize=True,
            record=True,
        )
        save_video(scenario_name, frequency=int(1.0 / args.dt))
