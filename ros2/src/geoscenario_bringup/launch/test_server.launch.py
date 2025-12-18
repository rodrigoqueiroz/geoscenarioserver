"""
Launch file for testing GeoScenario server with mock co-simulator.

Equivalent to test/test_ros2_server.bash

Usage:
    ros2 launch test_server.launch.py
    ros2 launch test_server.launch.py time_mode:=fastest
    ros2 launch test_server.launch.py time_mode:=2xrealtime scenario_file:=path/to/scenario.osm
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    time_mode_arg = DeclareLaunchArgument(
        'time_mode',
        default_value='realtime',
        description='Time control mode: `realtime`, `fastest`, or `2xrealtime`'
    )

    scenario_file_arg = DeclareLaunchArgument(
        'scenario_file',
        default_value='scenarios/test_scenarios/gs_all_vehicles_peds.osm',
        description='Path to scenario file'
    )

    dashboard_position_arg = DeclareLaunchArgument(
        'dashboard_position',
        default_value='[0, 0, 1920, 1080]',
        description='Dashboard window position [x, y, width, height]'
    )

    # Get launch configurations
    time_mode = LaunchConfiguration('time_mode')
    scenario_file = LaunchConfiguration('scenario_file')
    dashboard_position = LaunchConfiguration('dashboard_position')

    # Calculate parameters based on time_mode
    # realtime: factor=1.0, max_time=30.0
    # fastest: factor=0.0, max_time=-1.0
    # 2xrealtime: factor=0.5, max_time=30.0
    real_time_factor = PythonExpression([
        "0.0 if '", time_mode, "' == 'fastest' else (0.5 if '", time_mode, "' == '2xrealtime' else 1.0)"
    ])
    max_simulation_time = PythonExpression([
        "-1.0 if '", time_mode, "' == 'fastest' else 30.0"
    ])

    # Mock co-simulator node
    mock_co_simulator_node = Node(
        package='geoscenario_client',
        executable='mock_co_simulator',
        name='mock_co_simulator',
        output='screen',
        parameters=[{
            'target_delta_time': 0.025,
            'max_simulation_time': max_simulation_time,
            'real_time_factor': real_time_factor,
        }]
    )

    # GeoScenario server node
    geoscenario_server_node = Node(
        package='geoscenario_server',
        executable='geoscenario_server',
        name='geoscenario_server',
        output='screen',
        arguments=['--ros-args', '--log-level', 'INFO'],
        parameters=[{
            'scenario_files': scenario_file,
            'dashboard_position': dashboard_position,
        }]
    )

    # Shutdown when server exits
    shutdown_on_server_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=geoscenario_server_node,
            on_exit=[EmitEvent(event=Shutdown())]
        )
    )

    return LaunchDescription([
        time_mode_arg,
        scenario_file_arg,
        dashboard_position_arg,
        mock_co_simulator_node,
        geoscenario_server_node,
        shutdown_on_server_exit,
    ])
