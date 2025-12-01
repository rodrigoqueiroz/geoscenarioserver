# Native ROS2 GeoScenarioServer

The `ros2/geoscenario_server` wraps GSServer into a ROS2 node
1. publishes server state to `/tick`
2. subscribes to `/tick_from_client` and updates server state for external vehicles. 

## Usage

```
ros2 run geoscenario_server geoscenario_server [--ros-args -p scenario_files -p no_dashboard -p map_path -p btree_locations -p dashboard_position -p wgs84 -p write_trajectories]
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