# SPDX-License-Identifier: Apache-2.0

"""Tests for the Scenario dataclass."""

from pink_bench import Scenario


class TestScenario:
    """Tests for the Scenario dataclass."""

    def test_scenario_basic_creation(self):
        """Test creating a basic Scenario."""
        scenario = Scenario(
            name="test_scenario",
            duration=10.0,
            robot_description="panda_description",
        )
        assert scenario.name == "test_scenario"
        assert scenario.duration == 10.0
        assert scenario.robot_description == "panda_description"
        assert scenario.n_joints is None
        assert scenario.root_joint is None
        assert scenario.initial_configuration == {}
        assert scenario.trajectories == []

    def test_scenario_with_initial_configuration(
        self, sample_initial_configuration
    ):
        """Test creating a Scenario with initial configuration."""
        scenario = Scenario(
            name="test_scenario",
            duration=5.0,
            robot_description="panda_description",
            initial_configuration=sample_initial_configuration,
        )
        assert scenario.initial_configuration == sample_initial_configuration
        assert len(scenario.initial_configuration) == 7

    def test_scenario_with_root_joint(self, free_flyer_joint):
        """Test creating a Scenario with a root joint."""
        scenario = Scenario(
            name="floating_robot",
            duration=10.0,
            robot_description="atlas_drc_description",
            root_joint=free_flyer_joint,
        )
        assert scenario.root_joint is not None
        # JointModelFreeFlyer is a specific type, not a base JointModel
        assert hasattr(scenario.root_joint, "nq")
        assert hasattr(scenario.root_joint, "nv")

    def test_scenario_with_n_joints(self):
        """Test creating a Scenario with specified number of joints."""
        scenario = Scenario(
            name="test_scenario",
            duration=10.0,
            robot_description="panda_description",
            n_joints=7,
        )
        assert scenario.n_joints == 7

    def test_scenario_with_all_fields(
        self, sample_initial_configuration, free_flyer_joint
    ):
        """Test creating a Scenario with all fields populated."""
        scenario = Scenario(
            name="complete_scenario",
            duration=15.0,
            robot_description="test_robot_description",
            n_joints=10,
            root_joint=free_flyer_joint,
            initial_configuration=sample_initial_configuration,
            trajectories=[],
        )
        assert scenario.name == "complete_scenario"
        assert scenario.duration == 15.0
        assert scenario.robot_description == "test_robot_description"
        assert scenario.n_joints == 10
        assert scenario.root_joint is not None
        assert len(scenario.initial_configuration) == 7
        assert isinstance(scenario.trajectories, list)

    def test_scenario_immutable_modification(self):
        """Test that modifying a Scenario after creation works as expected."""
        scenario = Scenario(
            name="test",
            duration=10.0,
            robot_description="panda_description",
        )
        # Dataclasses are mutable by default
        scenario.duration = 20.0
        assert scenario.duration == 20.0
