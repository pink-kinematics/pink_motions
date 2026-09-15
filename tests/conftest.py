# SPDX-License-Identifier: Apache-2.0

"""Pytest configuration and fixtures for pink_bench tests."""

import pinocchio as pin
import pytest


@pytest.fixture
def sample_robot_description():
    """Fixture providing a sample robot description name."""
    return "panda_description"


@pytest.fixture
def sample_initial_configuration():
    """Fixture providing a sample robot configuration."""
    return {
        "panda_joint1": 0.0,
        "panda_joint2": -0.785,
        "panda_joint3": 0.0,
        "panda_joint4": -2.356,
        "panda_joint5": 0.0,
        "panda_joint6": 1.571,
        "panda_joint7": 0.785,
    }


@pytest.fixture
def free_flyer_joint():
    """Fixture providing a free flyer root joint."""
    return pin.JointModelFreeFlyer()
