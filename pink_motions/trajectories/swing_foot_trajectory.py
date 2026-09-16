# SPDX-License-Identifier: Apache-2.0

"""Swing-foot trajectory, used by the walking scenarios."""

import numpy as np
import pink
import pinocchio as pin
from meshcat import Visualizer

from .trajectory import Trajectory


class SwingFootTrajectory(Trajectory):
    """Step a foot frame forward, lifting it between footholds.

    Footholds are spaced by ``stride`` along the x-axis of the world frame,
    starting from where the foot stands at reset time. During the swing
    phase of each walking cycle, the target moves in a straight line from
    one foothold to the next, with a vertical clearance peaking at
    ``height`` (see ``height_polynomial``). Outside of it, it rests on a
    foothold: the current one before take-off, the next one after landing.

    Pair one instance per foot with a ``LIPMWalkingTrajectory`` sharing the
    same phase durations and stride to get a walking scenario.
    """

    def __init__(
        self,
        task: pink.tasks.FrameTask,
        cycle_duration: float,
        height: float,
        is_lead_foot: bool,
        lateral_offset: float,
        stride: float,
        swing_end: float,
        swing_start: float,
    ):
        """Initialize swing foot trajectory.

        Args:
            task: Frame task whose target to update.
            cycle_duration: Duration of a full walking cycle, in seconds.
            height: Peak clearance of the foot above its footholds, in
                meters.
            is_lead_foot: If set, this foot takes the first step. Its first
                stride is halved, as the center of mass is slower then than
                in the periodic regime.
            lateral_offset: Lateral offset, in meters, from the initial
                placement of the foot frame to the first foothold.
            stride: Distance between consecutive footholds of this foot, in
                meters.
            swing_end: Time in the walking cycle, in seconds, at which the
                foot lands on its next foothold.
            swing_start: Time in the walking cycle, in seconds, at which the
                foot takes off.

        Raises:
            AssertionError: if the swing phase is shorter than 10 ms.
        """
        super().__init__(task)
        swing_duration = swing_end - swing_start

        # Lead foot goes first while the center of mass velocity is lower than
        # in the period regime, hence we scale down the first stride by half.
        first_stride_factor = 0.5 if is_lead_foot else 1.0
        transform_first_to_initial = pin.SE3(
            rotation=np.eye(3),
            translation=first_stride_factor * np.array([stride, 0.0, 0.0]),
        )

        transform_step_to_initial = pin.SE3(
            rotation=np.eye(3),
            translation=np.array([stride, 0.0, 0.0]),
        )
        Delta_first = pin.log6(transform_first_to_initial)
        Delta_step = pin.log6(transform_step_to_initial)
        assert swing_duration > 0.01
        self.Delta_first = Delta_first
        self.Delta_step = Delta_step
        self.cycle_duration = cycle_duration
        self.height = height
        self.lateral_offset = lateral_offset
        self.nb_steps = None
        self.swing_duration = swing_duration
        self.swing_start = swing_start
        self.t = None
        self.transform_first_to_initial = transform_first_to_initial
        self.transform_initial_to_world = None
        self.transform_step_to_initial = transform_step_to_initial
        self.transform_foot_to_model = None

    def reset(self, configuration: pink.Configuration, viewer: Visualizer):
        """Reset the trajectory to a robot configuration.

        The first foothold is the current position of the foot frame,
        shifted laterally by ``lateral_offset`` and aligned with the world
        frame. The orientation of the foot frame at reset time is kept as an
        offset applied to all targets, so that stepping does not rotate the
        foot.

        Args:
            configuration: Initial configuration of the robot.
            viewer: MeshCat viewer, unused by this trajectory.
        """
        super().reset(configuration, viewer)
        self.nb_steps = 0
        self.t = 0.0
        transform_orig_to_world = (  # foot frame in robot model
            configuration.get_transform_frame_to_world(self.task.frame)
        )
        transform_foot_to_world = pin.SE3(
            rotation=np.eye(3),
            translation=transform_orig_to_world.translation,
        )
        transform_initial_to_foot = pin.SE3(
            rotation=np.eye(3),
            translation=np.array([0.0, self.lateral_offset, 0.0]),
        )
        self.transform_initial_to_world = (
            transform_foot_to_world * transform_initial_to_foot
        )
        self.transform_orig_to_foot = pin.SE3(
            rotation=transform_orig_to_world.rotation,
            translation=np.zeros(3),
        )

    def height_polynomial(self, x: float):
        """Quintic polynomial with boundary conditions.

        Conditions:

        - Initial position: 0
        - Initial velocity: 0
        - Maximum value: self.height
        - Target position: 0
        - Target velocity: 0
        - Target acceleration: 0

        Args:
            x: Value of the argument to the polynomial.

        Returns:
            Value of the polynomial at x.
        """
        init_accel = 3125 * self.height / 54
        return 0.5 * init_accel * x**2 * (1 + x * (-3 + x * (3 - x)))

    def step(self, dt: float):
        """Update target frame (in place).

        Args:
            dt: Timestep in seconds.
        """
        self.t += dt
        swing_time = (self.t % self.cycle_duration) - self.swing_start
        s = np.clip(swing_time / self.swing_duration, 0.0, 1.0)
        nb_steps = int(self.t // self.cycle_duration)
        T = self.transform_initial_to_world.copy()
        if nb_steps > 0:
            T *= self.transform_first_to_initial  # historical artifact
            for _ in range(1, nb_steps):
                T *= self.transform_step_to_initial
        Delta = self.Delta_first if nb_steps == 0 else self.Delta_step
        T *= pin.exp6(s * Delta)
        transform_mid_to_world = T
        h = self.height_polynomial(s)
        transform_swing_to_mid = pin.SE3(
            rotation=np.eye(3),
            translation=np.array([0.0, 0.0, h]),
        )
        self.task.set_target(
            transform_mid_to_world
            * transform_swing_to_mid
            * self.transform_orig_to_foot
        )
