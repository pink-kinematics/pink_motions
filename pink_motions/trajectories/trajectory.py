# SPDX-License-Identifier: Apache-2.0

"""Base class for the task targets that scenarios move around."""

import abc

from meshcat import Visualizer
from pink import Configuration, Task


class Trajectory:
    """Time-varying target for a single inverse kinematics task.

    A trajectory owns exactly one task and moves its target as time advances.
    Scenarios of the library are lists of trajectories: they are reset once
    from the initial configuration of the robot, then unrolled open-loop by
    repeated calls to the ``step`` method, which subclasses should implement.

    Attributes:
        task: Task whose target this trajectory moves.
    """

    __metaclass__ = abc.ABCMeta

    def __init__(self, task: Task):
        """Initialize the trajectory.

        Args:
            task: Task whose target this trajectory moves.
        """
        self.task = task

    def reset(self, configuration: Configuration, viewer: Visualizer):
        """Reset the trajectory to a robot configuration.

        The task target is initialized from the configuration, so that
        trajectories start from where the robot stands.

        Args:
            configuration: Initial configuration of the robot.
            viewer: MeshCat viewer for annotations, ``None`` when
                visualization is disabled.
        """
        self.task.set_target_from_configuration(configuration)

    @abc.abstractmethod
    def step(self, dt: float):
        """Move the task target by one timestep.

        Args:
            dt: Duration of the timestep, in seconds.
        """
        pass
