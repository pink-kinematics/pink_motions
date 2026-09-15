# SPDX-License-Identifier: Apache-2.0

import numpy as np
import pink
import pinocchio as pin
from meshcat import Visualizer

from .trajectory import Trajectory


class SwingFootTrajectory(Trajectory):
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
