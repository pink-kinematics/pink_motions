# SPDX-License-Identifier: Apache-2.0

"""Tests for the scenario library."""

from pink_motions import SCENARIOS, Scenario


class TestLibrary:
    """Tests for the SCENARIOS library."""

    def test_scenarios_dict_exists(self):
        """Test that SCENARIOS is a dictionary."""
        assert isinstance(SCENARIOS, dict)
        assert len(SCENARIOS) > 0

    def test_scenarios_contain_scenario_objects(self):
        """Test that all values in SCENARIOS are Scenario objects."""
        for name, scenario in SCENARIOS.items():
            assert isinstance(scenario, Scenario)
            assert isinstance(name, str)

    def test_scenario_names_match_keys(self):
        """Test that scenario names match their dictionary keys."""
        for key, scenario in SCENARIOS.items():
            assert scenario.name == key

    def test_scenarios_have_valid_durations(self):
        """Test that all scenarios have positive durations."""
        for scenario in SCENARIOS.values():
            assert scenario.duration > 0

    def test_scenarios_have_robot_descriptions(self):
        """Test that all scenarios have robot descriptions."""
        for scenario in SCENARIOS.values():
            assert scenario.robot_description is not None
            assert isinstance(scenario.robot_description, str)
            assert len(scenario.robot_description) > 0

    def test_common_scenarios_exist(self):
        """Test that some common expected scenarios exist."""
        # These are common robot names that should be in the library
        common_scenarios = ["panda", "ur3", "ur5"]
        for scenario_name in common_scenarios:
            assert scenario_name in SCENARIOS, (
                f"Expected scenario '{scenario_name}' not found"
            )

    def test_scenario_trajectories_list(self):
        """Test that scenario trajectories are lists."""
        for scenario in SCENARIOS.values():
            assert isinstance(scenario.trajectories, list)

    def test_get_scenario_by_name(self):
        """Test getting a scenario by name from the library."""
        # Get first scenario name
        first_scenario_name = list(SCENARIOS.keys())[0]
        scenario = SCENARIOS[first_scenario_name]
        assert scenario.name == first_scenario_name
        assert isinstance(scenario, Scenario)
