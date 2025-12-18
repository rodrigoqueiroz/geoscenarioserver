# Native ROS2 GeoScenarioServer

## Folder Structure

```
ros2/src/
├── geoscenario_msgs/      # ROS2 message definitions (Tick, Vehicle, Pedestrian)
├── geoscenario_server/    # Native ROS2 server node
├── geoscenario_client/    # Shared memory client and mock co-simulator
└── geoscenario_bringup/   # Launch files and integration tests
    ├── launch/            # Launch files
    └── test/              # Integration tests (launch_testing)
```

## geoscenario_server

The `ros2/geoscenario_server` wraps GSServer into a ROS2 node
1. publishes server state to `/tick`
2. subscribes to `/tick_from_client` and updates server state for external vehicles. 

## Usage

```
ros2 run geoscenario_server geoscenario_server --ros-args \
        -p scenario_files:=[<path>, ...] \
        -p no_dashboard:=true \
        -p map_path:=<path> \
        -p btree_locations:=[<path>, ...] \
        -p dashboard_position:=[<x>, <y>, <w>, <h>] \
        -p wgs84:=true \
        -p write_trajectories:=true
```

# GeoScenarioServer shared memory client for ROS2

The `ros2/geoscenario_client` interfaces between GSS shared memory and ROS2 topics:
1. reads from GSS shared memory block for server and publishes to `/tick`
2. subscribes to `/tick_from_client` and writes to GSS shared memory block for client

## Usage

```
ros2 run geoscenario_client geoscenario_client [--ros-args -p wgs84:=true]
```

The client publishes the coordinates in a local cartesian frame defined by the scenario's origin. 
The argument `wgs84:=true` (`false` by default) changes the coordinate frame to WGS84 instead of local cartesian coordinates.

A ROS2 subscriber to `/tick` can determine that WGS84 coordinates are used if `/tick/origin=(0,0,0)`.

The client can execute a round-trip test: store values from server shared memory, convert to WGS84, send to mock co-simulator, receive, convert from WGS84, compare with stored.
The argument `roundtriptest:=true` (`false` by default) enables this round-trip test.

## Testing

Run all bringup tests:
```bash
pixi run -e humble ros_test_bringup
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