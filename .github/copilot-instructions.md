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

### Server Implementations

GeoScenario Server has **two implementations**:

1. **Standalone Python Server** (`GSServer.py`)
   - Command: `pixi run gss` or `gsserver`
   - Timer-based simulation loop at fixed rate (default 40Hz)
   - Uses shared memory for co-simulation with external clients
   - Suitable for standalone simulations and legacy co-sim workflows

2. **Native ROS2 Server** (`geoscenario_server.py`)
   - Command: `pixi run -e humble ros_gss` or `ros2 run geoscenario_server geoscenario_server`
   - Event-driven simulation controlled by ROS2 topics `/gs/tick` and `/gs/tick_from_client`
   - No shared memory - direct ROS2 topic communication
   - Suitable for ROS2-integrated systems and co-simulation

Both implementations support the same core features (scenario loading, behavior trees, trajectory generation) but differ in their external interface and execution model.

### Running Scenarios

**Standalone Python Server:**
```bash
# Basic scenario execution
pixi run gss --scenario scenarios/test_scenarios/gs_all_vehicles_peds.osm

# Multiple scenario files (merged at runtime)
pixi run gss -s scenario1.osm scenario2.osm scenario3.osm

# Set origin to vehicle starting position (VUT-relative coordinates)
pixi run gss -s <file.osm> --origin-from-vid 10

# Run without dashboard
pixi run gss -s <file.osm> --no-dash

# Write trajectory CSV files
pixi run gss -s <file.osm> --write-trajectories

# Debug mode with file logging
pixi run gss -s <file.osm> --debug --file-log

# Wait for user input before starting
pixi run gss -s <file.osm> --wait-for-input

# Custom dashboard position (x y width height)
pixi run gss -s <file.osm> --dash-pos 100 100 1200 800

# Custom map path prefix
pixi run gss -s <file.osm> --map-path /custom/maps/

# Custom behavior tree locations (colon-separated)
pixi run gss -s <file.osm> --btree-locations /path/to/btrees:/another/path

# Run test suite
pixi run test_scenarios_ci
# or with options:
bash scripts/run_scenarios.bash --no-dash --non-interactive
```

**Native ROS2 Server:**
```bash
# Build ROS2 packages first
pixi run -e humble ros_build

# Basic ROS2 server
pixi run -e humble ros_gss --scenario <file.osm>

# With ROS2 parameters
ros2 run geoscenario_server geoscenario_server --ros-args \
  -p scenario_files:="['/path/to/scenario.osm']" \
  -p origin_from_vid:=10 \
  -p no_dashboard:=true \
  -p write_trajectories:=true \
  -p wgs84:=true

# Run ROS2 client (in separate terminal)
pixi run -e humble ros_client

# Run mock co-simulator for testing
pixi run -e humble ros_mock_co_simulator
```

### Command-Line Parameters

**Standalone Python Server (`gsserver`):**

| Parameter | Short | Type | Default | Description |
|-----------|-------|------|---------|-------------|
| `--scenario` | `-s` | list[str] | `[]` | GeoScenario .osm file(s). Multiple files are merged at runtime |
| `--verify_map` | | str | `""` | Verify Lanelet map file and exit |
| `--no-dash` | `-n` | flag | False | Run without the dashboard GUI |
| `--map-path` | `-m` | str | `""` | Prefix for lanelet map file path |
| `--btree-locations` | `-b` | str | `""` | Colon-separated additional btree search paths |
| `--wait-for-input` | `-wi` | flag | False | Wait for [ENTER] before starting simulation |
| `--wait-for-client` | `-wc` | flag | False | Wait for valid client state before starting |
| `--dash-pos` | `-dp` | float[4] | `[]` | Dashboard window position: x y width height |
| `--debug` | `-d` | flag | False | Set log level to DEBUG (default: INFO) |
| `--file-log` | `-fl` | flag | False | Write logs to `$GSS_OUTPUTS/GSServer.log` |
| `--write-trajectories` | `-wt` | flag | False | Write agent trajectories to CSV files |
| `--origin-from-vid` | `-ofv` | int | 0 | Set origin to vehicle's starting position (0 = use scenario origin) |
| `--quiet` | `-q` | flag | True | Don't print messages to stdout |

**Native ROS2 Server Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `scenario_files` | string_array | `['']` | GeoScenario .osm file paths |
| `no_dashboard` | bool | False | Run without the dashboard |
| `map_path` | string | `""` | Prefix for lanelet map file path |
| `btree_locations` | string | `""` | Additional btree search locations |
| `dashboard_position` | integer_array | `[0.0]` | Dashboard window position [x, y, width, height] |
| `wgs84` | bool | False | Use WGS84+origin(0,0,0) instead of local+origin=(lat,lon,alt) |
| `write_trajectories` | bool | False | Write agent trajectories to CSV files |
| `origin_from_vid` | int | 0 | Set origin to vehicle's starting position (0 = use scenario origin) |

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
- `<tag k='gs' v='origin'>`: lat/lon/altitude for coordinate projection (can be set to vehicle position via `--origin-from-vid` CLI argument)
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
