# GeoScenario Server - AI Agent Instructions

## Project Overview
GeoScenario Server is an autonomous vehicle scenario simulation framework with behavior tree-based planning and Frenet frame motion planning. It simulates vehicles (SDV, TV, PV, EV) and pedestrians (SP, TP, PP) on Lanelet2 maps with real-time co-simulation support.

GeoScenario Server works with scenario files defined according to [GeoScenario2](https://geoscenario2.readthedocs.io/) format. The format is an extension of OpenStreetMap (.osm) XML files with custom tags for defining agents, routes, and global configuration.

## Architecture

### Core Components
- **GSServer.py**: Main entry point - orchestrates simulation loop via `SimTraffic` and `TickSync`
- **SimTraffic**: Central traffic coordinator managing all actors, shared memory, and requirements checking
- **Actor hierarchy**: Base classes for vehicles/pedestrians with shared state management (`Actor.py`)
  - Vehicle types: SDV (simulated driver-vehicle), TV (trajectory), PV (path), EV (external)
  - Pedestrian types: SP (simulated), TP (trajectory), PP (path)
- **ScenarioSetup.py**: Parses `.osm` GeoScenario files and instantiates actors

### Planning Architecture (SDV vehicles only)
1. **SVPlanner** (multiprocess): Spawns separate process per SDV vehicle
2. **Behavior Layer**: Either `btree` (behavior trees via py_trees==0.7.6) or `ruleEngine` (external rule engine)
3. **Maneuver Planning**: Frenet frame trajectory generation (`sv/maneuvers/FrenetTrajectory.py`)
4. **Requirements Checking**: Hard/soft requirements validated per tick (`requirements/RequirementsChecker.py`)

Behavior trees are parsed from `.btree` files using custom ANTLR4 DSL (`sv/planners/btree/parser/BTreeDSL.g4`). Trees located in `btrees/{sdv,sp}/` by agent type.

### Coordinate Systems
- **Simulation frame**: Local Cartesian (ENU) from origin lat/lon via Lanelet2 projectors (LocalCartesian or UTM)
- **Frenet frame**: Path-following coordinates (s=longitudinal, d=lateral) relative to reference path
- **WGS84**: GPS coordinates for external interfaces
- State vectors: `[x, x_vel, x_acc, y, y_vel, y_acc]` (Cartesian) or `[s, s_vel, s_acc, d, d_vel, d_acc]` (Frenet)

### Shared Memory for Co-Simulation
- Server writes: tick_count, sim_time, origin, all actor states (`shm/SimSharedMemoryServer.py`)
- Client writes: external vehicle states (e.g., from Unreal/Carla)
- Layout: Space-separated UTF8 string (see `shm/README.md`)
- Keys configurable in `SimConfig.py` (SHM_KEY, SEM_KEY, etc.)

## Development Workflows

### Running Scenarios
```bash
# Using pixi (recommended)
pixi run gss --scenario scenarios/test_scenarios/gs_intersection_greenlight.osm

# Run test suite
pixi run test_scenarios_ci  # or bash scripts/run_scenarios.bash --no-dash --non-interactive

# With ROS2 client
pixi run -e humble ros_gss --scenario <file.osm>
pixi run -e humble ros_client  # in another terminal
```

### Environment Setup
- **Pixi (recommended)**: `pixi shell` (default env) or `pixi shell -e humble` (with ROS2)
- **Alternative using micromamba**: `bash scripts/setup-conda-forge-env.bash --ros2` defined in `scripts/conda-environment.yml`
- **Native setup using apt-get and pip**: `bash scripts/install_dependencies.bash --ros2` defined in `requirements.txt`.

### Regenerating ANTLR Parsers
```bash
pixi run -e antlr regenerate  # regenerates sv/planners/btree/parser and sp/btree/parser
```

### Output Files
All outputs write to `outputs/` (override with `GSS_OUTPUTS` env var):
- Trajectories, logs, requirement violation events (collisions, timeouts, etc.)
- Behavior tree graphs (if `GENERATE_GRAPH_TREE=True` in `SimConfig.py`)

## Key Conventions

### File Organization Patterns
- **Agents by type**: `sv/` (simulated vehicles), `sp/` (simulated pedestrians)
- **Planners**: `sv/planners/{btree,ruleEngine}/` - behavior selection layers
- **Maneuvers**: `sv/maneuvers/` - motion primitives and trajectory generation
- **Scenarios**: `scenarios/{test_scenarios,nhtsa,carla_scenarios}/` - `.osm` files + maps in `scenarios/maps/`
- **Behavior trees**: `btrees/{sdv,sp}/<name>.btree` - loaded by `btype` tag

### GeoScenario (.osm) File Structure

Format documented in [GeoScenario2](https://geoscenario2.readthedocs.io/).

Parsed by `gsc/GSParser.py`. Key elements:
- `<tag k='gs' v='globalconfig'>`: timeout, lanelet map, scenario name
- `<tag k='gs' v='origin'>`: lat/lon/altitude for coordinate projection
- `<node>` with `gs='vehicle'`: must have `btype` (sdv/tv/pv/ev), optional `btree`, `route`/`trajectory`/`path`, `vid`
- `<node>` with `gs='pedestrian'`: `btype` (sp/tp/pp), optional `btree`, `destination`/`route`, `pid`
- Multiple `.osm` files can be merged (see `--scenario` accepting multiple files)

### Critical Configuration Points
- **SimConfig.py**: All tunable parameters (rates, dashboard, plotting, collision detection)
  - `TRAFFIC_RATE=40`: Global simulation tick rate (Hz)
  - `PLANNER_RATE=5`: SDV planner tick rate
  - `COLL_TYPE_RADIUS=True`: Circle-based collision (vs. rectangular bounding box)
- **Frenet paths**: Use `SDVRoute.get_global_path()` and `update_reference_path(s)` for dynamic reference paths
- **Actor states**: Read from `Actor.state` (VehicleState/PedestrianState), never modify directly during planning

### Behavior Tree Conventions
- Leaves return `SUCCESS`, `FAILURE`, `RUNNING` (py_trees standard)
- Maneuver leaves (e.g., `lane_keep`, `lane_change`) set `mconfig` (ManeuverConfig) in blackboard
- Condition leaves (e.g., `is_at_intersection`) query `traffic_state` from blackboard
- Reconfig syntax: `<maneuver_id>:<param>=<value>` (e.g., `lk:target_speed=15.0`)

### Multiprocessing Patterns
- **SDV planners**: Separate process per vehicle using `multiprocessing.Process`
- **Shared arrays**: `traffic_state_sharr`, `mplan_sharr` for inter-process communication
- **Dashboard**: Separate process with `Manager().dict()` for debug data
- Always use `Value('b', False)` for boolean flags, `Array('f', size)` for float arrays

### Requirements/Safety Checks
- Inherit from `HardRequirements` and `SoftRequirements` in `requirements/`
- Violations logged to `RequirementViolationEvents` (timestamped CSV)
- Hard requirement violations raise exceptions (e.g., `ScenarioCompletion`, collision detection)
- Check `RequirementsChecker.analyze(traffic_state)` called every SDV tick

## Common Pitfalls
1. **Don't** assume paths are static - SDV routes update reference paths dynamically via `sdv_route.update_reference_path(s)`
2. **Don't** use `python -c` for quick Python checks - use the Pylance MCP tool `pylanceRunCodeSnippet` instead
3. **Don't** modify `Actor.state` directly - use motion plan application or trajectory following
4. **Don't** forget coordinate frame conversions - use `frenet_to_sim_frame` and `sim_to_frenet_frame` from `util/Transformations.py`
5. **py_trees version locked at 0.7.6** - API incompatible with newer versions

## Testing
- Regression tests: Outputs compared with `outputs/regressions/` baselines
- Run single scenario: `pixi run gss --scenario <file.osm> --single`
- ROS2 client roundtrip test: `bash scripts/pixi_test_ros2_client.bash --roundtriptest`

## External Integrations
- **Lanelet2 maps**: Required for all scenarios, stored in `scenarios/maps/`
- **ROS2 client**: `clients/ros2_client/` - reads server ShM, publishes `/tick` topic
- **Unreal/Carla**: Optional co-sim via ShM or CarlaSync (`shm/CarlaSync.py`)
