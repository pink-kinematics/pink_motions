# SPDX-License-Identifier: Apache-2.0

"""Back-and-forth trajectory, used by the arm scenarios."""

from typing import Optional

import numpy as np
import pink
import pinocchio as pin
from meshcat import Visualizer
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

from .trajectory import Trajectory


class BackAndForthTrajectory(Trajectory):
    """Sweep a frame target back and forth between two positions.

    The target oscillates between ``start_position`` and ``stop_position``
    following a squared sine of time, while rocking in yaw and roll with
    the same phase.
    """

    def __init__(
        self,
        task: pink.tasks.FrameTask,
        position_scale: float = 1.0,
        start_position: Optional[NDArray[float]] = None,
        stop_position: Optional[NDArray[float]] = None,
        target_orientation: Optional[Rotation] = None,
    ):
        """Prepare a task-target update function.

        Args:
            task: Frame task whose target to move.
            position_scale: Scaling factor applied to the position trajectory
                in the world frame.
            start_position: Initial position, in meters. Defaults to a
                position suitable for a fixed-base arm.
            stop_position: Position at the other end of the trajectory, in
                meters. Defaults to a position suitable for a fixed-base arm.
            target_orientation: Orientation of the task frame with respect to
                the default orientation of the trajectory in the world frame.
        """
        super().__init__(task)
        if start_position is None:
            start_position = np.array([0.4, 0.8, 0.4])
        if stop_position is None:
            stop_position = np.array([-0.4, 0.6, 0.6])
        Delta = stop_position - start_position
        R_target_to_ref = (
            Rotation.identity()
            if target_orientation is None
            else target_orientation
        )
        self.Delta = Delta
        self.R_target_to_ref = R_target_to_ref
        self.position_scale = position_scale
        self.start_position = start_position
        self.t = 0.0
        self.transform_target_to_world = pin.SE3.Identity()

    def reset(self, configuration: pink.Configuration, viewer: Visualizer):
        """Reset the trajectory to a robot configuration.

        The target transform is re-anchored on the current placement of the
        task frame, whose translation and rotation ``step`` then overwrites.

        Args:
            configuration: Initial configuration of the robot.
            viewer: MeshCat viewer, unused by this trajectory.
        """
        super().reset(configuration, viewer)
        self.transform_target_to_world = (
            configuration.get_transform_frame_to_world(self.task.frame)
        )

    def step(self, dt: float) -> pin.SE3:
        """Update target frame (in place).

        Args:
            dt: Time step.

        Returns:
            Transform from target to world.
        """
        self.t += dt
        s2 = np.sin(self.t) ** 2

        # Translation
        position = self.start_position + self.Delta * s2
        self.transform_target_to_world.translation = (
            self.position_scale * position
        )

        # Rotation
        yaw_pitch_roll = np.array(
            [np.pi * 0.25 * (1 + s2), 0.0, np.pi / 4.0 * s2]
        )
        R_ref_to_world = Rotation.from_euler("ZYX", yaw_pitch_roll)
        R_target_to_ref = self.R_target_to_ref
        R_target_to_world = R_ref_to_world * R_target_to_ref
        self.transform_target_to_world.rotation = np.array(
            R_target_to_world.as_matrix()
        )

        self.task.set_target(self.transform_target_to_world)
