# SPDX-License-Identifier: Apache-2.0

from pathlib import Path
from typing import List, Optional

import meshcat_shapes
import numpy as np
import pink
import pinocchio as pin
from numpy.typing import NDArray
from pink import solve_ik
from pink.limits import VelocityLimit
from pink.tasks import FrameTask
from pink.utils import custom_configuration_vector
from pink.visualization import start_meshcat_visualizer
from robot_descriptions.loaders.pinocchio import load_robot_description

from .scenario import Scenario
from .trajectories import LIPMWalkingTrajectory


def make_limits(
    model, scenario: Scenario
) -> Optional[List[pink.limits.Limit]]:
    """Assemble the limits a scenario enforces on top of its model's.

    Robots whose joints are all continuous carry neither a configuration
    nor a velocity limit, which would leave their IK problem an
    unconstrained QP. Scenarios bound those with an explicit uniform
    velocity limit, a pure tangent-space box that never touches the cos/sin
    configuration of a continuous joint.

    Args:
        model: Robot model of the scenario.
        scenario: Scenario whose limits are assembled.

    Returns:
        List of limits, or ``None`` when the scenario adds none, so that
        ``solve_ik`` falls back to the limits of the model.
    """
    if scenario.velocity_limit is None:
        return None
    v_max = scenario.velocity_limit * np.ones(model.nv)
    limits = [
        model.configuration_limit,
        VelocityLimit(model, velocity_limit=v_max),
    ]
    floating_base_limit = getattr(model, "floating_base_velocity_limit", None)
    if floating_base_limit is not None:
        limits.append(floating_base_limit)
    return limits


class Scene:
    configuration: pink.Configuration
    robot: pin.RobotWrapper

    def __init__(
        self,
        scenario: Scenario,
        visualize: bool = False,
        record: bool = False,
    ):
        """Create a new scene.

        Args:
            scenario: Scene description.
            visualize: If set, visualize scenario in MeshCat.
            record: If set, each frame will be rendered to an image as well.

        Raises:
            ValueError: if the scenario is empty.
            IndexError: if the initial configuration does not match the robot
                model.
        """
        robot = load_robot_description(
            description_name=scenario.robot_description,
            root_joint=scenario.root_joint,
        )

        visualizer = None
        viewer = None
        if visualize:
            visualizer = start_meshcat_visualizer(robot)
            viewer = visualizer.viewer
            viewer["/Background"].set_property("top_color", [1] * 3)
            viewer["/Background"].set_property("bottom_color", [1] * 3)
            viewer["/Grid"].set_property("visible", False)
            viewer["/Cameras/default/rotated/<object>"].set_property(
                "zoom", 2.0
            )

        if len(scenario.trajectories) < 1:
            robot_frames = [
                frame.name
                for frame in robot.model.frames
                if frame.name != "universe"
            ]
            raise ValueError(
                "Scenario is empty, maybe add a task for one of:\n\n- "
                + ("\n- ".join(robot_frames))
            )

        try:
            q_init = custom_configuration_vector(
                robot, **scenario.initial_configuration
            )
        except IndexError as exn:
            queried_joints = list(scenario.initial_configuration.keys())
            valid_joints = list(robot.model.names)
            raise IndexError(
                f"One joint in {queried_joints} not found, "
                f"names must be in {valid_joints}"
            ) from exn

        if record:
            videos_dir = Path("videos")
            if not videos_dir.exists():
                videos_dir.mkdir(parents=True, exist_ok=True)

        if visualize:
            visualizer.display(q_init)
        configuration = pink.Configuration(robot.model, robot.data, q_init)
        for trajectory in scenario.trajectories:
            trajectory.reset(configuration, viewer)
            task = trajectory.task
            if viewer is not None and hasattr(task, "frame"):
                frame = task.frame
                meshcat_shapes.frame(viewer[f"{frame}_current"], opacity=1.0)
                meshcat_shapes.frame(viewer[f"{frame}_target"], opacity=0.5)

        self.configuration = configuration
        self.limits = make_limits(configuration.model, scenario)
        self.record = record
        self.recording_index = 0
        self.q_init = q_init
        self.robot = robot
        self.scenario = scenario
        self.trajectories = scenario.trajectories
        self.viewer = viewer
        self.visualizer = visualizer

    def plot_mpc_axis(self, plot_axis: int) -> None:
        for trajectory in self.trajectories:
            if isinstance(trajectory, LIPMWalkingTrajectory):
                trajectory.init_plot(plot_axis)

    def reset(self):
        model = self.robot.model
        data = self.robot.data
        if self.visualizer is not None:
            self.visualizer.display(self.q_init)
        self.configuration = pink.Configuration(model, data, self.q_init)
        for trajectory in self.scenario.trajectories:
            trajectory.reset(self.configuration, self.viewer)

    @property
    def tasks(self) -> List[pink.Task]:
        return [trajectory.task for trajectory in self.trajectories]

    @property
    def frame_tasks(self) -> List[FrameTask]:
        return [
            trajectory.task
            for trajectory in self.trajectories
            if isinstance(trajectory.task, FrameTask)
        ]

    def step_targets(self, dt: float) -> None:
        """Advance targets for a given duration.

        Args:
            dt: Duration in seconds.
        """
        for trajectory in self.trajectories:
            trajectory.step(dt)

        if self.viewer is None:
            return
        for task in self.frame_tasks:
            if not hasattr(task, "frame"):
                continue
            frame = task.frame
            target = task.transform_target_to_world
            current = self.configuration.get_transform_frame_to_world(frame)
            self.viewer[f"{frame}_target"].set_transform(target.np)
            self.viewer[f"{frame}_current"].set_transform(current.np)

    def step_velocity(self, velocity: NDArray[float], dt: float) -> None:
        """Advance the scene with a given robot velocity.

        Args:
            velocity: Velocity vector in the robot's tangent space.
            dt: Duration in seconds.
        """
        self.configuration.integrate_inplace(velocity, dt)
        if self.visualizer is not None:
            self.visualizer.display(self.configuration.q)
            if self.record:
                fname = f"{self.scenario.name}_{self.recording_index:04d}.png"
                self.recording_index += 1
                with open(f"videos/{fname}", "wb") as fp:
                    self.viewer.get_image().save(fp)

    def step(self, dt: float, solver: str) -> None:
        """Advance the scene.

        Args:
            dt: Duration inseconds.
            solver: Backend quadratic programming (QP) solver.
        """
        self.step_targets(dt)
        velocity = solve_ik(
            self.configuration,
            self.tasks,
            dt,
            solver=solver,
            limits=self.limits,
        )
        self.step_velocity(velocity, dt)
