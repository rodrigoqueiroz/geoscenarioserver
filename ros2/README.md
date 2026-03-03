# GeoScenario ROS2

Native ROS2 server for GeoScenario Server, providing event-driven simulation controlled by ROS2 topics.

## Quick Start

All pixi commands must be executed from the `geoscenarioserver/` directory.

## Run Native Server with Mock Co-Simulator

The first run will build the native GSS server as well as the ROS2 package.

```bash
cd geoscenarioserver/
pixi run -e humble ros2 launch geoscenario_bringup test_server.launch.py
```

With options:
```bash
pixi run -e humble ros2 launch geoscenario_bringup test_server.launch.py time_mode:=fastest
pixi run -e humble ros2 launch geoscenario_bringup test_server.launch.py scenario_files:="['path/to/scenario.osm']"
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

To build a single `ros-humble-geoscenario-*` conda package:

```bash
pixi build --path ros2/geoscenario_msgs/package.xml
pixi build --path ros2/geoscenario_server/package.xml
pixi build --path ros2/geoscenario_client/package.xml
pixi build --path ros2/geoscenario_bringup/package.xml
```

The package is output to `ros-humble-geoscenario-*.conda`.

### Using in Another Project

Add to your `pixi.toml`:

```toml
[workspace]
channels = ["./local-channel", "https://prefix.dev/robostack-humble", "https://prefix.dev/conda-forge"]

[dependencies]
ros-humble-geoscenario-msgs = "*"
ros-humble-geoscenario-server = "*"
ros-humble-geoscenario-client = "*"
ros-humble-geoscenario-bringup = "*"
```

Copy the `.conda` files to `local-channel/linux-64/`, index, and install:

```bash
pixi exec rattler-index fs local-channel
pixi install
```

Run the server with the mock co-simulator:

```bash
pixi run -e humble ros2 launch geoscenario_bringup test_server.launch.py
```
