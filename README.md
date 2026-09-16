# Pink motions

[![PyPI version](https://img.shields.io/pypi/v/pink_motions)](https://pypi.org/project/pink_motions/)

A library of robot motions performed by inverse kinematics and model predictive control:

<p align="center">
    <img src="https://github.com/user-attachments/assets/5b169c2b-3c84-47b8-96d0-5b9095255f52" alt="Sample scenarios from pink motions" />
</p>

## Usage

You can try out a scenario from the library directly using [pixi](https://pixi.sh):

```console
pixi run scenario --robot ur5 --qpsolver quadprog
```

Or equivalently `uv run examples/play_scenario.py` with the same arguments. In Python, this example simply does:

```py
import pink_motions

pink_motions.play_scenario(
    name="ur5",
    dt=0.005,  # timestep in seconds
    qpsolver="quadprog",
)
```

This will open a MeshCat tab in your web browser playing the scenario for its prescribed duration.

## See also

Pink motions are specified using a combination of several open-source libraries:

- [robot\_descriptions.py](https://github.com/robot-descriptions/robot_descriptions.py) to load robot descriptions
- [Pinocchio](https://github.com/stack-of-tasks/pinocchio/) for forward kinematics functions: Jacobians, Jlog6, ...
- [MeshCat](https://github.com/meshcat-dev/meshcat-python) for (optional) visualization
- [Pink](https://github.com/pink-kinematics/pink) for differential inverse kinematics
- [qpmpc](https://github.com/stephane-caron/qpmpc) for linear model predictive control, used in biped and humanoid scenarios

The library is also related to the following projects:

- [ik\_qpbenchmark](https://github.com/qpsolvers/ik_qpbenchmark): Test set for QP solvers with differential IK problems generated from pink motions.
- [LoIK](https://github.com/Simple-Robotics/LoIK): Constrained differential inverse kinematics solver that was evaluated on pink motions.
