"""
Launch testing for heartbeat timeout detection.

Usage:
    launch_test src/geoscenario_bringup/test/test_heartbeat_timeout.py
    colcon test --packages-select geoscenario_bringup
"""

import unittest

import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers

from launch import LaunchDescription
from launch_ros.actions import Node

import pytest


STALL_AFTER_TICKS = 5
TARGET_DELTA_TIME = 0.1
TEST_TIMEOUT = 30.0
SCENARIO_FILE = 'scenarios/test_scenarios/gs_straight_tv.osm'


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    mock_co_simulator_node = Node(
        package='geoscenario_client',
        executable='mock_co_simulator',
        name='mock_co_simulator',
        output='screen',
        parameters=[{
            'target_delta_time': TARGET_DELTA_TIME,
            'real_time_factor': 0.0,
            'max_simulation_time': -1.0,
            'stall_after_ticks': STALL_AFTER_TICKS,
        }]
    )

    geoscenario_server_node = Node(
        package='geoscenario_server',
        executable='geoscenario_server',
        name='geoscenario_server',
        output='screen',
        arguments=['--ros-args', '--log-level', 'INFO'],
        parameters=[{
            'scenario_files': [SCENARIO_FILE],
            'no_dashboard': True,
        }]
    )

    return LaunchDescription([
        mock_co_simulator_node,
        geoscenario_server_node,
        launch_testing.actions.ReadyToTest(),
    ]), {
        'mock_co_simulator': mock_co_simulator_node,
        'geoscenario_server': geoscenario_server_node,
    }


class TestHeartbeatTimeout(unittest.TestCase):

    def test_server_detects_stall(self, proc_output, geoscenario_server):
        proc_output.assertWaitFor(
            expected_output='Co-simulator stalled',
            process=geoscenario_server,
            timeout=TEST_TIMEOUT,
        )

    def test_server_exits_interrupted(self, proc_output, geoscenario_server):
        proc_output.assertWaitFor(
            expected_output='Shutting down due to interruption or error',
            process=geoscenario_server,
            timeout=TEST_TIMEOUT,
        )


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_server_exit_code(self, proc_info, geoscenario_server):
        # Allow SIGINT (-2) since launch_testing may send it while server is already shutting down
        launch_testing.asserts.assertExitCodes(
            proc_info,
            process=geoscenario_server,
            allowable_exit_codes=[0, -2],
        )

    def test_mock_cosim_exit_code(self, proc_info, mock_co_simulator):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            process=mock_co_simulator,
        )
