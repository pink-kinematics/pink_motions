# SPDX-License-Identifier: Apache-2.0

"""Benchmark scenarios: a robot, an initial pose and a task trajectory.

Each entry of ``SCENARIOS`` specifies a Pink task and the robot
description it runs on; the descriptions come from ``robot_descriptions``,
pinned exactly so that every machine loads the same robots.
"""

from typing import Dict

import numpy as np
import pinocchio as pin
from pink.tasks import FrameTask
from scipy.spatial.transform import Rotation

from .scenario import Scenario
from .trajectories import (
    BackAndForthTrajectory,
    LIPMWalkingTrajectory,
    SwingFootTrajectory,
)

# Dictionary mapping scenario names to their corresponding scenario
SCENARIOS: Dict[str, Scenario] = {}

# =============================================================================

SCENARIOS["edo"] = Scenario(
    name="edo",
    duration=10.0,  # [s]
    robot_description="edo_description",
    initial_configuration={
        "joint_1": [-0.7623581513637019, 0.6471553515573514],
        "joint_2": [0.0342308422806942, 0.9994139529928298],
        "joint_3": [0.9310376721972858, 0.36492307812669145],
        "joint_4": [-0.03470501799257229, 0.9993975994198381],
        "joint_5": [0.06148178569673815, 0.9981082055706887],
        "joint_6": [0.3342551633135528, 0.9424826183003218],
    },
    trajectories=[
        BackAndForthTrajectory(
            FrameTask(
                "link_6",
                position_cost=1.0,  # [cost] / [m]
                orientation_cost=1.0,  # [cost] / [rad]
            ),
            position_scale=0.6,
            target_orientation=Rotation.from_euler("ZYX", [np.pi, 0.0, 0.0]),
        ),
    ],
)

# =============================================================================

SCENARIOS["fanuc"] = Scenario(
    name="fanuc",
    duration=10.0,  # [s]
    robot_description="fanuc_m710ic_description",
    initial_configuration={
        "joint_1": 1.0,
        "joint_2": 2.0,
        "joint_3": -2.5,
    },
    trajectories=[
        BackAndForthTrajectory(
            FrameTask(
                "tool0",
                position_cost=1.0,  # [cost] / [m]
                orientation_cost=1.0,  # [cost] / [rad]
            ),
            position_scale=2.2,
        ),
    ],
)

# =============================================================================

SCENARIOS["gen2"] = Scenario(
    name="gen2",
    duration=10.0,  # [s]
    robot_description="gen2_description",
    initial_configuration={
        "j2s6s200_joint_2": 2.5,
        "j2s6s200_joint_3": 4.0,
        "j2s6s200_joint_5": 2.0,
    },
    trajectories=[
        BackAndForthTrajectory(
            FrameTask(
                "j2s6s200_end_effector",
                position_cost=1.0,  # [cost] / [m]
                orientation_cost=1.0,  # [cost] / [rad]
            ),
            position_scale=0.6,
            target_orientation=Rotation.from_euler(
                "ZYX", [0.5 * np.pi, 0.5 * np.pi, 0.0]
            ),
        ),
    ],
)

# =============================================================================

SCENARIOS["gen3"] = Scenario(
    name="gen3",
    duration=10.0,  # [s]
    robot_description="gen3_description",
    initial_configuration={
        "j2n6s300_joint_2": 2.5,
        "j2n6s300_joint_3": 4.0,
        "j2n6s300_joint_5": [0.0, 1.0],
    },
    trajectories=[
        BackAndForthTrajectory(
            FrameTask(
                "j2n6s300_end_effector",
                position_cost=1.0,  # [cost] / [m]
                orientation_cost=1.0,  # [cost] / [rad]
            ),
            position_scale=0.7,
            target_orientation=Rotation.from_euler("ZYX", [0.0, 0.0, np.pi]),
        ),
    ],
)

# =============================================================================

SCENARIOS["iiwa14"] = Scenario(
    name="iiwa14",
    duration=10.0,  # [s]
    robot_description="iiwa14_description",
    trajectories=[
        BackAndForthTrajectory(
            FrameTask(
                "iiwa_link_ee",
                position_cost=1.0,  # [cost] / [m]
                orientation_cost=1.0,  # [cost] / [rad]
            ),
        ),
    ],
)

# =============================================================================

SCENARIOS["panda"] = Scenario(
    name="panda",
    duration=10.0,  # [s]
    robot_description="panda_description",
    trajectories=[
        BackAndForthTrajectory(
            FrameTask(
                "panda_hand",
                position_cost=1.0,  # [cost] / [m]
                orientation_cost=1.0,  # [cost] / [rad]
            ),
            position_scale=0.7,
            target_orientation=Rotation.from_euler(
                "ZYX", [0.0, 0.0, np.pi * 0.5]
            ),
        ),
    ],
)

# =============================================================================

SCENARIOS["poppy_ergo_jr"] = Scenario(
    name="poppy_ergo_jr",
    duration=10.0,  # [s]
    robot_description="poppy_ergo_jr_description",
    trajectories=[
        BackAndForthTrajectory(
            FrameTask(
                "fixed_tip",
                position_cost=1.0,  # [cost] / [m]
                orientation_cost=1.0,  # [cost] / [rad]
            ),
            position_scale=0.08,
        ),
    ],
)

# =============================================================================

SCENARIOS["ur10"] = Scenario(
    name="ur10",
    duration=10.0,  # [s]
    robot_description="ur10_description",
    trajectories=[
        BackAndForthTrajectory(
            FrameTask(
                "ee_link",
                position_cost=1.0,  # [cost] / [m]
                orientation_cost=1.0,  # [cost] / [rad]
            ),
            position_scale=1.0,
        ),
    ],
)

# =============================================================================

SCENARIOS["ur3"] = Scenario(
    name="ur3",
    duration=10.0,  # [s]
    robot_description="ur3_description",
    trajectories=[
        BackAndForthTrajectory(
            FrameTask(
                "ee_link",
                position_cost=1.0,  # [cost] / [m]
                orientation_cost=1.0,  # [cost] / [rad]
            ),
            position_scale=0.5,
        ),
    ],
)

# =============================================================================

SCENARIOS["ur5"] = Scenario(
    name="ur5",
    duration=10.0,  # [s]
    robot_description="ur5_description",
    trajectories=[
        BackAndForthTrajectory(
            FrameTask(
                "ee_link",
                position_cost=1.0,  # [cost] / [m]
                orientation_cost=1.0,  # [cost] / [rad]
            ),
            position_scale=0.9,
        ),
    ],
)

# =============================================================================

SCENARIOS["z1"] = Scenario(
    name="z1",
    duration=10.0,  # [s]
    robot_description="z1_description",
    trajectories=[
        BackAndForthTrajectory(
            FrameTask(
                "link06",
                position_cost=1.0,  # [cost] / [m]
                orientation_cost=1.0,  # [cost] / [rad]
            ),
            position_scale=0.7,
        ),
    ],
)

# =============================================================================

SCENARIOS["atlas_drc"] = Scenario(
    name="atlas_drc",
    duration=10.0,  # [s]
    robot_description="atlas_drc_description",
    root_joint=pin.JointModelFreeFlyer(),
    initial_configuration={
        "root_joint": pin.SE3ToXYZQUAT(
            pin.SE3(
                rotation=np.eye(3),
                translation=np.array([0.0, 0.0, 0.94]),
            )
        ),
    },
    trajectories=[
        LIPMWalkingTrajectory(
            FrameTask(
                "root_joint",
                position_cost=1.0,
                orientation_cost=1.0,
            ),
            dsp_duration=0.1,  # [s]
            foot_size=(0.09, 0.05),  # [m]
            foot_spacing=0.32,
            lateral_offset=0.05,  # should match swing-foot trajectories
            left_foot_frame="l_foot",
            nb_timesteps=16,
            sampling_period=0.1,  # [s]
            ssp_duration=0.7,  # [s]
            stride=0.5,
            transform_target_to_root=pin.SE3(
                rotation=np.eye(3),
                translation=np.array([-0.1, 0.0, -0.1]),
            ),
        ),
        SwingFootTrajectory(
            FrameTask(
                "l_foot",
                position_cost=1.0,  # [cost] / [m]
                orientation_cost=1.0,  # [cost] / [rad]
            ),
            cycle_duration=1.6,  # [s]
            height=0.15,  # [m]
            is_lead_foot=False,
            lateral_offset=0.05,  # [m]
            stride=0.5,  # [m]
            swing_end=1.5,  # [s]
            swing_start=0.8,  # [s]
        ),
        SwingFootTrajectory(
            FrameTask(
                "r_foot",
                position_cost=1.0,  # [cost] / [m]
                orientation_cost=1.0,  # [cost] / [rad]
            ),
            cycle_duration=1.6,  # [s]
            height=0.15,  # [m]
            is_lead_foot=True,
            lateral_offset=-0.05,  # [m]
            stride=0.5,  # [m]
            swing_end=0.7,
            swing_start=0.0,
        ),
    ],
)

# =============================================================================

SCENARIOS["atlas_v4"] = Scenario(
    name="atlas_v4",
    duration=10.0,  # [s]
    robot_description="atlas_v4_description",
    root_joint=pin.JointModelFreeFlyer(),
    trajectories=[
        LIPMWalkingTrajectory(
            FrameTask(
                "root_joint",
                position_cost=1.0,
                orientation_cost=1.0,
            ),
            dsp_duration=0.1,  # [s]
            foot_size=(0.09, 0.05),  # [m]
            foot_spacing=0.32,
            lateral_offset=0.05,  # should match swing-foot trajectories
            left_foot_frame="l_foot",
            nb_timesteps=16,
            sampling_period=0.1,  # [s]
            ssp_duration=0.7,  # [s]
            stride=0.5,
            transform_target_to_root=pin.SE3(
                rotation=np.eye(3),
                translation=np.array([-0.1, 0.0, -0.1]),
            ),
        ),
        SwingFootTrajectory(
            FrameTask(
                "l_foot",
                position_cost=1.0,  # [cost] / [m]
                orientation_cost=1.0,  # [cost] / [rad]
            ),
            cycle_duration=1.6,  # [s]
            height=0.15,  # [m]
            is_lead_foot=False,
            lateral_offset=0.05,  # [m]
            stride=0.5,  # [m]
            swing_end=1.5,  # [s]
            swing_start=0.8,  # [s]
        ),
        SwingFootTrajectory(
            FrameTask(
                "r_foot",
                position_cost=1.0,  # [cost] / [m]
                orientation_cost=1.0,  # [cost] / [rad]
            ),
            cycle_duration=1.6,  # [s]
            height=0.15,  # [m]
            is_lead_foot=True,
            lateral_offset=-0.05,  # [m]
            stride=0.5,  # [m]
            swing_end=0.7,
            swing_start=0.0,
        ),
    ],
)

# =============================================================================

SCENARIOS["draco3"] = Scenario(
    name="draco3",
    duration=10.0,  # [s]
    robot_description="draco3_description",
    root_joint=pin.JointModelFreeFlyer(),
    initial_configuration={
        "l_shoulder_aa": 0.5,
        "r_shoulder_aa": -0.5,
        "l_wrist_ps": np.pi / 2,
        "r_wrist_ps": -np.pi / 2,
    },
    trajectories=[
        LIPMWalkingTrajectory(
            FrameTask(
                "root_joint",
                position_cost=1.0,
                orientation_cost=1.0,
            ),
            dsp_duration=0.1,  # [s]
            foot_size=(0.09, 0.05),  # [m]
            foot_spacing=0.21,
            lateral_offset=0.0,  # should match swing-foot trajectories
            left_foot_frame="l_foot_contact",
            nb_timesteps=16,
            sampling_period=0.1,  # [s]
            ssp_duration=0.7,  # [s]
            stride=0.5,
            transform_target_to_root=pin.SE3(
                rotation=np.eye(3),
                translation=np.array([-0.1, 0.0, -0.1]),
            ),
        ),
        SwingFootTrajectory(
            FrameTask(
                "l_foot_contact",
                position_cost=1.0,  # [cost] / [m]
                orientation_cost=1.0,  # [cost] / [rad]
            ),
            cycle_duration=1.6,  # [s]
            height=0.15,  # [m]
            is_lead_foot=False,
            lateral_offset=0.0,  # [m]
            stride=0.5,  # [m]
            swing_end=1.5,  # [s]
            swing_start=0.8,  # [s]
        ),
        SwingFootTrajectory(
            FrameTask(
                "r_foot_contact",
                position_cost=1.0,  # [cost] / [m]
                orientation_cost=1.0,  # [cost] / [rad]
            ),
            cycle_duration=1.6,  # [s]
            height=0.15,  # [m]
            is_lead_foot=True,
            lateral_offset=-0.0,  # [m]
            stride=0.5,  # [m]
            swing_end=0.7,
            swing_start=0.0,
        ),
    ],
)

# =============================================================================

SCENARIOS["ergocub"] = Scenario(
    name="ergocub",
    duration=10.0,  # [s]
    robot_description="ergocub_description",
    root_joint=pin.JointModelFreeFlyer(),
    trajectories=[
        LIPMWalkingTrajectory(
            FrameTask(
                "root_joint",
                position_cost=1.0,
                orientation_cost=1.0,
            ),
            dsp_duration=0.1,  # [s]
            foot_size=(0.09, 0.05),  # [m]
            foot_spacing=0.15,
            lateral_offset=0.0,  # should match swing-foot trajectories
            left_foot_frame="l_sole",
            nb_timesteps=16,
            sampling_period=0.1,  # [s]
            ssp_duration=0.7,  # [s]
            stride=0.5,
            transform_target_to_root=pin.SE3(
                rotation=np.eye(3),
                translation=np.array([-0.1, 0.0, -0.1]),
            ),
        ),
        SwingFootTrajectory(
            FrameTask(
                "l_sole",
                position_cost=1.0,  # [cost] / [m]
                orientation_cost=1.0,  # [cost] / [rad]
            ),
            cycle_duration=1.6,  # [s]
            height=0.15,  # [m]
            is_lead_foot=False,
            lateral_offset=0.0,  # [m]
            stride=0.5,  # [m]
            swing_end=1.5,  # [s]
            swing_start=0.8,  # [s]
        ),
        SwingFootTrajectory(
            FrameTask(
                "r_sole",
                position_cost=1.0,  # [cost] / [m]
                orientation_cost=1.0,  # [cost] / [rad]
            ),
            cycle_duration=1.6,  # [s]
            height=0.15,  # [m]
            is_lead_foot=True,
            lateral_offset=-0.0,  # [m]
            stride=0.5,  # [m]
            swing_end=0.7,
            swing_start=0.0,
        ),
    ],
)

# =============================================================================

SCENARIOS["h1"] = Scenario(
    name="h1",
    duration=10.0,  # [s]
    robot_description="h1_description",
    root_joint=pin.JointModelFreeFlyer(),
    trajectories=[
        LIPMWalkingTrajectory(
            FrameTask(
                "root_joint",
                position_cost=1.0,
                orientation_cost=0.1,
            ),
            dsp_duration=0.1,  # [s]
            foot_size=(0.09, 0.05),  # [m]
            foot_spacing=0.42,
            lateral_offset=0.0,  # should match swing-foot trajectories
            left_foot_frame="left_ankle_link",
            nb_timesteps=16,
            sampling_period=0.1,  # [s]
            ssp_duration=0.7,  # [s]
            stride=0.5,
            transform_target_to_root=pin.SE3(
                rotation=np.eye(3),
                translation=np.array([-0.1, 0.0, -0.1]),
            ),
        ),
        SwingFootTrajectory(
            FrameTask(
                "left_ankle_link",
                position_cost=1.0,  # [cost] / [m]
                orientation_cost=1.0,  # [cost] / [rad]
            ),
            cycle_duration=1.6,  # [s]
            height=0.15,  # [m]
            is_lead_foot=False,
            lateral_offset=0.0,  # [m]
            stride=0.5,  # [m]
            swing_end=1.5,  # [s]
            swing_start=0.8,  # [s]
        ),
        SwingFootTrajectory(
            FrameTask(
                "right_ankle_link",
                position_cost=1.0,  # [cost] / [m]
                orientation_cost=1.0,  # [cost] / [rad]
            ),
            cycle_duration=1.6,  # [s]
            height=0.15,  # [m]
            is_lead_foot=True,
            lateral_offset=-0.0,  # [m]
            stride=0.5,  # [m]
            swing_end=0.7,
            swing_start=0.0,
        ),
    ],
)

# =============================================================================

SCENARIOS["icub"] = Scenario(
    name="icub",
    duration=10.0,  # [s]
    robot_description="icub_description",
    root_joint=pin.JointModelFreeFlyer(),
    initial_configuration={
        "l_elbow": np.pi / 2,
        "l_shoulder_pitch": np.pi / 4,
        "l_shoulder_roll": np.pi / 4,
        "r_elbow": np.pi / 2,
        "r_shoulder_pitch": np.pi / 4,
        "r_shoulder_roll": np.pi / 4,
        "root_joint": pin.SE3ToXYZQUAT(
            pin.SE3(
                rotation=np.array(
                    Rotation.from_euler("ZYX", [np.pi, 0.0, 0.0]).as_matrix()
                ),
                translation=np.zeros(3),
            )
        ),
    },
    trajectories=[
        LIPMWalkingTrajectory(
            FrameTask(
                "root_joint",
                position_cost=1.0,
                orientation_cost=1.0,
            ),
            dsp_duration=0.1,  # [s]
            foot_size=(0.07, 0.04),  # [m]
            foot_spacing=0.15,
            lateral_offset=0.0,  # should match swing-foot trajectories
            left_foot_frame="l_sole",
            nb_timesteps=16,
            sampling_period=0.1,  # [s]
            ssp_duration=0.7,  # [s]
            stride=0.3,
            transform_target_to_root=pin.SE3(
                rotation=np.eye(3),
                translation=np.array([0.0, 0.0, -0.05]),
            ),
        ),
        SwingFootTrajectory(
            FrameTask(
                "l_sole",
                position_cost=1.0,  # [cost] / [m]
                orientation_cost=1.0,  # [cost] / [rad]
            ),
            cycle_duration=1.6,  # [s]
            height=0.1,  # [m]
            is_lead_foot=False,
            lateral_offset=0.0,  # [m]
            stride=0.3,  # [m]
            swing_end=1.5,  # [s]
            swing_start=0.8,  # [s]
        ),
        SwingFootTrajectory(
            FrameTask(
                "r_sole",
                position_cost=1.0,  # [cost] / [m]
                orientation_cost=1.0,  # [cost] / [rad]
            ),
            cycle_duration=1.6,  # [s]
            height=0.1,  # [m]
            is_lead_foot=True,
            lateral_offset=-0.0,  # [m]
            stride=0.3,  # [m]
            swing_end=0.7,
            swing_start=0.0,
        ),
    ],
)

# =============================================================================

SCENARIOS["jaxon"] = Scenario(
    name="jaxon",
    duration=10.0,  # [s]
    robot_description="jaxon_description",
    root_joint=pin.JointModelFreeFlyer(),
    initial_configuration={
        "LARM_JOINT1": np.pi / 4,
        "RARM_JOINT1": np.pi / 4,
        "LARM_JOINT2": np.pi / 8,
        "RARM_JOINT2": -np.pi / 8,
        "LARM_JOINT4": -np.pi / 2,
        "RARM_JOINT4": -np.pi / 2,
    },
    trajectories=[
        LIPMWalkingTrajectory(
            FrameTask(
                "root_joint",
                position_cost=1.0,
                orientation_cost=1.0,
            ),
            dsp_duration=0.1,  # [s]
            foot_size=(0.09, 0.05),  # [m]
            foot_spacing=0.21,
            lateral_offset=0.0,  # should match swing-foot trajectories
            left_foot_frame="LLEG_LINK5",
            nb_timesteps=16,
            sampling_period=0.1,  # [s]
            ssp_duration=0.7,  # [s]
            stride=0.5,
            transform_target_to_root=pin.SE3(
                rotation=np.eye(3),
                translation=np.array([-0.1, 0.0, -0.1]),
            ),
        ),
        SwingFootTrajectory(
            FrameTask(
                "LLEG_LINK5",
                position_cost=1.0,  # [cost] / [m]
                orientation_cost=1.0,  # [cost] / [rad]
            ),
            cycle_duration=1.6,  # [s]
            height=0.15,  # [m]
            is_lead_foot=False,
            lateral_offset=0.0,  # [m]
            stride=0.5,  # [m]
            swing_end=1.5,  # [s]
            swing_start=0.8,  # [s]
        ),
        SwingFootTrajectory(
            FrameTask(
                "RLEG_LINK5",
                position_cost=1.0,  # [cost] / [m]
                orientation_cost=1.0,  # [cost] / [rad]
            ),
            cycle_duration=1.6,  # [s]
            height=0.15,  # [m]
            is_lead_foot=True,
            lateral_offset=-0.0,  # [m]
            stride=0.5,  # [m]
            swing_end=0.7,
            swing_start=0.0,
        ),
    ],
)

# =============================================================================

SCENARIOS["jvrc"] = Scenario(
    name="jvrc",
    duration=10.0,  # [s]
    robot_description="jvrc_description",
    root_joint=pin.JointModelFreeFlyer(),
    trajectories=[
        LIPMWalkingTrajectory(
            FrameTask(
                "root_joint",
                position_cost=1.0,
                orientation_cost=1.0,
            ),
            dsp_duration=0.1,  # [s]
            foot_size=(0.09, 0.05),  # [m]
            foot_spacing=0.21,
            lateral_offset=0.0,  # should match swing-foot trajectories
            left_foot_frame="l_ankle",
            nb_timesteps=16,
            sampling_period=0.1,  # [s]
            ssp_duration=0.7,  # [s]
            stride=0.5,
            transform_target_to_root=pin.SE3(
                rotation=np.eye(3),
                translation=np.array([-0.1, 0.0, -0.1]),
            ),
        ),
        SwingFootTrajectory(
            FrameTask(
                "l_ankle",
                position_cost=1.0,  # [cost] / [m]
                orientation_cost=1.0,  # [cost] / [rad]
            ),
            cycle_duration=1.6,  # [s]
            height=0.15,  # [m]
            is_lead_foot=False,
            lateral_offset=0.0,  # [m]
            stride=0.5,  # [m]
            swing_end=1.5,  # [s]
            swing_start=0.8,  # [s]
        ),
        SwingFootTrajectory(
            FrameTask(
                "r_ankle",
                position_cost=1.0,  # [cost] / [m]
                orientation_cost=1.0,  # [cost] / [rad]
            ),
            cycle_duration=1.6,  # [s]
            height=0.15,  # [m]
            is_lead_foot=True,
            lateral_offset=-0.0,  # [m]
            stride=0.5,  # [m]
            swing_end=0.7,
            swing_start=0.0,
        ),
    ],
)

# =============================================================================

SCENARIOS["r2"] = Scenario(
    name="r2",
    duration=10.0,  # [s]
    robot_description="r2_description",
    root_joint=pin.JointModelFreeFlyer(),
    trajectories=[
        LIPMWalkingTrajectory(
            FrameTask(
                "root_joint",
                position_cost=1.0,
                orientation_cost=1.0,
            ),
            dsp_duration=0.1,  # [s]
            foot_size=(0.09, 0.05),  # [m]
            foot_spacing=0.5,
            lateral_offset=0.0,  # should match swing-foot trajectories
            left_foot_frame="r2/left_leg_foot",
            nb_timesteps=16,
            sampling_period=0.1,  # [s]
            ssp_duration=0.7,  # [s]
            stride=0.5,
            transform_target_to_root=pin.SE3(
                rotation=np.eye(3),
                translation=np.array([-0.1, 0.0, -0.1]),
            ),
        ),
        SwingFootTrajectory(
            FrameTask(
                "r2/left_leg_foot",
                position_cost=1.0,  # [cost] / [m]
                orientation_cost=1.0,  # [cost] / [rad]
            ),
            cycle_duration=1.6,  # [s]
            height=0.15,  # [m]
            is_lead_foot=False,
            lateral_offset=0.0,  # [m]
            stride=0.5,  # [m]
            swing_end=1.5,  # [s]
            swing_start=0.8,  # [s]
        ),
        SwingFootTrajectory(
            FrameTask(
                "r2/right_leg_foot",
                position_cost=1.0,  # [cost] / [m]
                orientation_cost=1.0,  # [cost] / [rad]
            ),
            cycle_duration=1.6,  # [s]
            height=0.15,  # [m]
            is_lead_foot=True,
            lateral_offset=-0.0,  # [m]
            stride=0.5,  # [m]
            swing_end=0.7,
            swing_start=0.0,
        ),
    ],
)

# =============================================================================

SCENARIOS["romeo"] = Scenario(
    name="romeo",
    duration=10.0,  # [s]
    robot_description="romeo_description",
    root_joint=pin.JointModelFreeFlyer(),
    trajectories=[
        LIPMWalkingTrajectory(
            FrameTask(
                "root_joint",
                position_cost=1.0,
                orientation_cost=1.0,
            ),
            dsp_duration=0.1,  # [s]
            foot_size=(0.09, 0.05),  # [m]
            foot_spacing=0.2,
            lateral_offset=0.0,  # should match swing-foot trajectories
            left_foot_frame="l_sole",
            nb_timesteps=16,
            sampling_period=0.1,  # [s]
            ssp_duration=0.7,  # [s]
            stride=0.5,
            transform_target_to_root=pin.SE3(
                rotation=np.eye(3),
                translation=np.array([-0.1, 0.0, -0.1]),
            ),
        ),
        SwingFootTrajectory(
            FrameTask(
                "l_sole",
                position_cost=1.0,  # [cost] / [m]
                orientation_cost=1.0,  # [cost] / [rad]
            ),
            cycle_duration=1.6,  # [s]
            height=0.15,  # [m]
            is_lead_foot=False,
            lateral_offset=0.0,  # [m]
            stride=0.5,  # [m]
            swing_end=1.5,  # [s]
            swing_start=0.8,  # [s]
        ),
        SwingFootTrajectory(
            FrameTask(
                "r_sole",
                position_cost=1.0,  # [cost] / [m]
                orientation_cost=1.0,  # [cost] / [rad]
            ),
            cycle_duration=1.6,  # [s]
            height=0.15,  # [m]
            is_lead_foot=True,
            lateral_offset=-0.0,  # [m]
            stride=0.5,  # [m]
            swing_end=0.7,
            swing_start=0.0,
        ),
    ],
)

# =============================================================================

SCENARIOS["sigmaban"] = Scenario(
    name="sigmaban",
    duration=10.0,  # [s]
    robot_description="sigmaban_description",
    root_joint=pin.JointModelFreeFlyer(),
    initial_configuration={
        "left_shoulder_roll": np.pi / 8,
        "left_shoulder_pitch": np.pi / 4,
        "left_elbow": -np.pi / 2,
        "right_shoulder_roll": -np.pi / 8,
        "right_shoulder_pitch": np.pi / 4,
        "right_elbow": -np.pi / 2,
    },
    trajectories=[
        LIPMWalkingTrajectory(
            FrameTask(
                "root_joint",
                position_cost=1.0,
                orientation_cost=1.0,
            ),
            dsp_duration=0.1,  # [s]
            foot_size=(0.07, 0.03),  # [m]
            foot_spacing=0.1,
            lateral_offset=0.0,  # should match swing-foot trajectories
            left_foot_frame="left_foot_asm",
            nb_timesteps=16,
            sampling_period=0.1,  # [s]
            ssp_duration=0.7,  # [s]
            stride=0.2,
            transform_target_to_root=pin.SE3(
                rotation=np.eye(3),
                translation=np.array([-0.1, 0.0, -0.05]),
            ),
        ),
        SwingFootTrajectory(
            FrameTask(
                "left_foot_asm",
                position_cost=1.0,  # [cost] / [m]
                orientation_cost=1.0,  # [cost] / [rad]
            ),
            cycle_duration=1.6,  # [s]
            height=0.05,  # [m]
            is_lead_foot=False,
            lateral_offset=0.0,  # [m]
            stride=0.2,  # [m]
            swing_end=1.5,  # [s]
            swing_start=0.8,  # [s]
        ),
        SwingFootTrajectory(
            FrameTask(
                "right_foot_asm",
                position_cost=1.0,  # [cost] / [m]
                orientation_cost=1.0,  # [cost] / [rad]
            ),
            cycle_duration=1.6,  # [s]
            height=0.05,  # [m]
            is_lead_foot=True,
            lateral_offset=-0.0,  # [m]
            stride=0.2,  # [m]
            swing_end=0.7,
            swing_start=0.0,
        ),
    ],
)

# =============================================================================

SCENARIOS["talos"] = Scenario(
    name="talos",
    duration=10.0,  # [s]
    robot_description="talos_description",
    root_joint=pin.JointModelFreeFlyer(),
    initial_configuration={
        "arm_left_2_joint": [-50.0],
        "arm_right_2_joint": [50.0],
        "leg_left_3_joint": [521.0360581588004],
        "leg_left_4_joint": [-1129.8926281203221],
        "leg_left_5_joint": [608.8565699615218],
        "leg_right_3_joint": [-672.8327536846762],
        "leg_right_4_joint": [1458.8755459660126],
        "leg_right_5_joint": [-786.0427922813365],
    },
    trajectories=[
        LIPMWalkingTrajectory(
            FrameTask(
                "root_joint",
                position_cost=1.0,
                orientation_cost=1.0,
            ),
            dsp_duration=0.1,  # [s]
            foot_size=(0.09, 0.05),  # [m]
            foot_spacing=0.21,
            lateral_offset=0.02,  # should match swing-foot trajectories
            left_foot_frame="leg_left_sole_fix_joint",
            nb_timesteps=16,
            sampling_period=0.1,  # [s]
            ssp_duration=0.7,  # [s]
            stride=0.5,
            transform_target_to_root=pin.SE3(
                rotation=np.eye(3),
                translation=np.array([-0.0, 0.0, -0.0]),
            ),
        ),
        SwingFootTrajectory(
            FrameTask(
                "leg_left_sole_fix_joint",
                position_cost=1.0,  # [cost] / [m]
                orientation_cost=1.0,  # [cost] / [rad]
            ),
            cycle_duration=1.6,  # [s]
            height=0.15,  # [m]
            is_lead_foot=False,
            lateral_offset=0.02,  # [m]
            stride=0.5,  # [m]
            swing_end=1.5,  # [s]
            swing_start=0.8,  # [s]
        ),
        SwingFootTrajectory(
            FrameTask(
                "leg_right_sole_fix_joint",
                position_cost=1.0,  # [cost] / [m]
                orientation_cost=1.0,  # [cost] / [rad]
            ),
            cycle_duration=1.6,  # [s]
            height=0.15,  # [m]
            is_lead_foot=True,
            lateral_offset=-0.02,  # [m]
            stride=0.5,  # [m]
            swing_end=0.7,
            swing_start=0.0,
        ),
    ],
)

# =============================================================================

SCENARIOS["valkyrie"] = Scenario(
    name="valkyrie",
    duration=10.0,  # [s]
    robot_description="valkyrie_description",
    root_joint=pin.JointModelFreeFlyer(),
    initial_configuration={
        "leftHipPitch": -0.39382399369154797,
        "leftKneePitch": -5.57880364237754,
        "leftAnklePitch": -0.31055767111050514,
        "rightHipPitch": 5.8563349714618775,
        "rightKneePitch": 0.8878146245997334,
        "rightAnklePitch": 5.822221018297585,
    },
    trajectories=[
        LIPMWalkingTrajectory(
            FrameTask(
                "root_joint",
                position_cost=1.0,
                orientation_cost=1.0,
            ),
            dsp_duration=0.1,  # [s]
            foot_size=(0.09, 0.05),  # [m]
            foot_spacing=0.32,
            lateral_offset=0.02,  # should match swing-foot trajectories
            left_foot_frame="leftFoot",
            nb_timesteps=16,
            sampling_period=0.1,  # [s]
            ssp_duration=0.7,  # [s]
            stride=0.5,
            transform_target_to_root=pin.SE3(
                rotation=np.eye(3),
                translation=np.array([-0.0, 0.0, -0.05]),
            ),
        ),
        SwingFootTrajectory(
            FrameTask(
                "leftFoot",
                position_cost=1.0,  # [cost] / [m]
                orientation_cost=1.0,  # [cost] / [rad]
            ),
            cycle_duration=1.6,  # [s]
            height=0.15,  # [m]
            is_lead_foot=False,
            lateral_offset=0.02,  # [m]
            stride=0.5,  # [m]
            swing_end=1.5,  # [s]
            swing_start=0.8,  # [s]
        ),
        SwingFootTrajectory(
            FrameTask(
                "rightFoot",
                position_cost=1.0,  # [cost] / [m]
                orientation_cost=1.0,  # [cost] / [rad]
            ),
            cycle_duration=1.6,  # [s]
            height=0.15,  # [m]
            is_lead_foot=True,
            lateral_offset=-0.02,  # [m]
            stride=0.5,  # [m]
            swing_end=0.7,
            swing_start=0.0,
        ),
    ],
)

# =============================================================================

SCENARIOS["bolt"] = Scenario(
    name="bolt",
    duration=10.0,  # [s]
    robot_description="bolt_description",
    root_joint=pin.JointModelFreeFlyer(),
    trajectories=[
        LIPMWalkingTrajectory(
            FrameTask(
                "root_joint",
                position_cost=1.0,
                orientation_cost=0.1,
            ),
            dsp_duration=0.1,  # [s]
            foot_size=(0.05, 0.01),  # [m]
            foot_spacing=0.22,
            lateral_offset=-0.02,  # should match swing-foot trajectories
            left_foot_frame="FL_FOOT",
            nb_timesteps=16,
            sampling_period=0.1,  # [s]
            ssp_duration=0.7,  # [s]
            stride=0.2,
            transform_target_to_root=pin.SE3(
                rotation=np.eye(3),
                translation=np.array([-0.05, 0.0, -0.1]),
            ),
        ),
        SwingFootTrajectory(
            FrameTask(
                "FL_FOOT",
                position_cost=1.0,  # [cost] / [m]
                orientation_cost=0.1,  # [cost] / [rad]
            ),
            cycle_duration=1.6,  # [s]
            height=0.05,  # [m]
            is_lead_foot=False,
            lateral_offset=-0.02,  # [m]
            stride=0.2,  # [m]
            swing_end=1.5,  # [s]
            swing_start=0.8,  # [s]
        ),
        SwingFootTrajectory(
            FrameTask(
                "FR_FOOT",
                position_cost=1.0,  # [cost] / [m]
                orientation_cost=0.1,  # [cost] / [rad]
            ),
            cycle_duration=1.6,  # [s]
            height=0.05,  # [m]
            is_lead_foot=True,
            lateral_offset=0.02,  # [m]
            stride=0.2,  # [m]
            swing_end=0.7,
            swing_start=0.0,
        ),
    ],
)

# =============================================================================

SCENARIOS["cassie"] = Scenario(
    name="cassie",
    duration=10.0,  # [s]
    robot_description="cassie_description",
    root_joint=pin.JointModelFreeFlyer(),
    initial_configuration={
        "LeftFootPitch": [-1.0],
        "RightFootPitch": [-1.0],
        "LeftAchillesSpring": [2.0],
        "RightAchillesSpring": [2.0],
        "root_joint": pin.SE3ToXYZQUAT(
            pin.SE3(
                rotation=np.eye(3),
                translation=np.array([0.0, 0.0, 1.15]),
            )
        ),
    },
    trajectories=[
        LIPMWalkingTrajectory(
            FrameTask(
                "root_joint",
                position_cost=1.0,
                orientation_cost=0.5,
            ),
            dsp_duration=0.1,  # [s]
            foot_size=(0.05, 0.01),  # [m]
            foot_spacing=0.24,
            lateral_offset=-0.02,  # should match swing-foot trajectories
            left_foot_frame="leftfoot",
            nb_timesteps=16,
            sampling_period=0.1,  # [s]
            ssp_duration=0.7,  # [s]
            stride=0.3,
            transform_target_to_root=pin.SE3(
                rotation=np.eye(3),
                translation=np.array([-0.05, 0.0, -0.15]),
            ),
        ),
        SwingFootTrajectory(
            FrameTask(
                "leftfoot",
                position_cost=1.0,  # [cost] / [m]
                orientation_cost=0.1,  # [cost] / [rad]
            ),
            cycle_duration=1.6,  # [s]
            height=0.05,  # [m]
            is_lead_foot=False,
            lateral_offset=-0.02,  # [m]
            stride=0.3,  # [m]
            swing_end=1.5,  # [s]
            swing_start=0.8,  # [s]
        ),
        SwingFootTrajectory(
            FrameTask(
                "rightfoot",
                position_cost=1.0,  # [cost] / [m]
                orientation_cost=0.1,  # [cost] / [rad]
            ),
            cycle_duration=1.6,  # [s]
            height=0.05,  # [m]
            is_lead_foot=True,
            lateral_offset=0.02,  # [m]
            stride=0.3,  # [m]
            swing_end=0.7,
            swing_start=0.0,
        ),
    ],
)

# =============================================================================

SCENARIOS["spryped"] = Scenario(
    name="spryped",
    duration=10.0,  # [s]
    robot_description="spryped_description",
    root_joint=pin.JointModelFreeFlyer(),
    trajectories=[
        LIPMWalkingTrajectory(
            FrameTask(
                "root_joint",
                position_cost=1.0,
                orientation_cost=0.5,
            ),
            dsp_duration=0.1,  # [s]
            foot_size=(0.05, 0.01),  # [m]
            foot_spacing=0.24,
            lateral_offset=-0.02,  # should match swing-foot trajectories
            left_foot_frame="left toe",
            nb_timesteps=16,
            sampling_period=0.1,  # [s]
            ssp_duration=0.7,  # [s]
            stride=0.3,
            transform_target_to_root=pin.SE3(
                rotation=np.eye(3),
                translation=np.array([-0.1, 0.0, -0.1]),
            ),
        ),
        SwingFootTrajectory(
            FrameTask(
                "left toe",
                position_cost=1.0,  # [cost] / [m]
                orientation_cost=0.1,  # [cost] / [rad]
            ),
            cycle_duration=1.6,  # [s]
            height=0.05,  # [m]
            is_lead_foot=False,
            lateral_offset=-0.02,  # [m]
            stride=0.3,  # [m]
            swing_end=1.5,  # [s]
            swing_start=0.8,  # [s]
        ),
        SwingFootTrajectory(
            FrameTask(
                "right toe",
                position_cost=1.0,  # [cost] / [m]
                orientation_cost=0.1,  # [cost] / [rad]
            ),
            cycle_duration=1.6,  # [s]
            height=0.05,  # [m]
            is_lead_foot=True,
            lateral_offset=0.02,  # [m]
            stride=0.3,  # [m]
            swing_end=0.7,
            swing_start=0.0,
        ),
    ],
)

# =============================================================================

__all__ = ["SCENARIOS"]
