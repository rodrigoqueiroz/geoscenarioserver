# GeoScenario ROS2

Native ROS2 server for GeoScenario Server, providing event-driven simulation controlled by ROS2 topics.

## Quick Start

All pixi commands must be executed from the `geoscenarioserver/` directory.

```bash
cd geoscenarioserver
```

### Build ROS2 Packages

```bash
pixi run ros_build          # Debug build
pixi run ros_build_release  # Release build
```

### Run Native Server with Mock Co-Simulator

```bash
pixi run ros2 launch geoscenario_bringup test_server.launch.py
```

With options:
```bash
pixi run ros2 launch geoscenario_bringup test_server.launch.py time_mode:=fastest
pixi run ros2 launch geoscenario_bringup test_server.launch.py scenario_files:="['path/to/scenario.osm']"
```

### Interactive Shell

```bash
pixi shell -e humble
```

Note: ROS2 tasks run in the `humble` environment (not `default`).

## Packages

### geoscenario_msgs

ROS2 message definitions for GeoScenario communication:
- `Tick.msg` - Complete simulation state per tick
- `Vehicle.msg` - Vehicle state
- `Pedestrian.msg` - Pedestrian state

### geoscenario_server

Native ROS2 server that wraps GSServer into a ROS2 node:
1. Publishes server state to `/gs/tick`
2. Subscribes to `/gs/tick_from_client` for external vehicle updates

#### Usage

```bash
ros2 run geoscenario_server geoscenario_server --ros-args \
        -p scenario_files:=[<path>, ...] \
        -p no_dashboard:=true \
        -p map_path:=<path> \
        -p btree_locations:=[<path>, ...] \
        -p dashboard_position:=[<x>, <y>, <w>, <h>] \
        -p wgs84:=true \
        -p write_trajectories:=true
```

### geoscenario_bringup

Launch files and integration tests for bringing up the system.

#### test_server.launch.py

Launches the native ROS2 server with the mock co-simulator:

```bash
ros2 launch geoscenario_bringup test_server.launch.py
```

**Parameters:**
- `time_mode` - Time control: `realtime` (default), `fastest`, or `2xrealtime`
- `scenario_files` - GeoScenario file paths as JSON array, e.g. `"['file1.osm', 'file2.osm']"`
- `dashboard_position` - Window position `[x, y, width, height]`

### geoscenario_client

Contains utilities for ROS2 co-simulation:
- `mock_co_simulator` - Test driver that responds to `/gs/tick` and publishes to `/gs/tick_from_client`
- `geoscenario_client` - Shared memory bridge for use with the standalone Python server (advanced use case)

## Testing

### Bringup Tests

Run all bringup integration tests:

```bash
pixi run ros_test_bringup
```

Or using colcon directly:

```bash
cd ros2
colcon test --packages-select geoscenario_bringup --event-handlers console_direct+
```

Run a single test file:

```bash
cd ros2
launch_test src/geoscenario_bringup/test/test_heartbeat_timeout.py
```

### Manual Server Test

```bash
bash test/test_ros2_server.bash [--fastest|--realtime|--2xrealtime]
```

### Shared Memory Client Test

For testing the shared memory bridge with the standalone Python server:

```bash
bash test/test_ros2_client.bash
bash geoscenarioserver/scripts/pixi_test_ros2_client.bash [--wgs84|--roundtriptest]
```

## Building Conda Package

To build a single `ros-humble-geoscenarioserver` conda package containing all ROS2 nodes (msgs, server, client, bringup):

```bash
cd ros2
pixi build -o dist/
```

The package is output to `ros2/dist/ros-humble-geoscenarioserver-*.conda`.

### Using in Another Project

Add to your `pixi.toml`:

```toml
[workspace]
channels = ["./local-channel", "https://prefix.dev/robostack-humble", "https://prefix.dev/conda-forge"]

[dependencies]
ros-humble-geoscenarioserver = "*"
```

Copy the `.conda` file to `local-channel/linux-64/` and index:

```bash
pixi exec rattler-index fs local-channel
pixi install
```

Run the server with the mock co-simulator:

```bash
pixi run ros2 launch geoscenario_bringup test_server.launch.py
```
