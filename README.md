#   GeoScenario Server

Includes: GeoScenario Parser, Checker, Sim Vehicle Planner with Behavior Trees and Maneuver Models.

## Dependencies

- Linux or Windows 10/11 + WSL2
- Python >= 3.11

GeoScenario Server can run on Linux natively or in WSL2 on Windows within a [conda](https://conda-forge.org/) environment.

### Deb packages for Linux native

Tested on native Ubuntu 20.04, 22.04, 24.04 and within Windows 10/11 WSL2 (See `WSL2-README.md` for details).

- libxft2
- python3
- python3-dev
- python3-tk
- python3-pip
- python3-pil
- python3-pil.imagetk

### Python packages

- antlr4-python3-runtime >= 4.13
- antlr-denter
- lanelet2
- matplotlib
- numpy
- scipy
- [py_trees==0.7.6](https://github.com/splintered-reality/py_trees)
- sysv-ipc
- tk
- pydot

#### Ubuntu native or Windows WSL2 installation

To automatically install the dependencies, execute

```bash
bash scripts/install_dependencies.bash
```

#### Conda-forge and robostack (ROS) using pixi (recommended) on Linux or WSL2

To install [pixi](https://pixi.sh/), execute
```bash
curl -fsSL https://pixi.sh/install.sh | bash
```
Re-open the terminal or source your `.bashrc` to make `pixi` available.

All pixi commands must be executed in geoscenarioserver as the working directory.
```bash
cd geoscenarioserver
```

Pixi project provides the following tasks:
```bash
cd geoscenarioserver
pixi run gss <parameters>
pixi run test_scenarios_ci
pixi run rqt
pixi run rqt_topic
pixi run ros_gss <parameters>
pixi run ros_build
pixi run ros_build_release
pixi run ros_server <ros parameters>
pixi run ros_client
pixi run ros_client_wgs84
pixi run ros_client_wgs84_roundtriptest
pixi run ros_mock_co_simulator
pixi run regenerate
```

The task `gss` runs in the environment `default` whereas `ros_gss` runs in `humble`.

To run automated test of ROS2 client using the mock co-simulator, execute:
```bash
bash geoscenarioserver/scripts/pixi_test_ros2_client.bash [--wgs84|--roundtriptest]
```
By default, the client will use local coordinates.
Use `--wgs84` flag to convert to and from WGS84 coordinates.
Use `--roundtriptest` flag to enable round-trip testing in addition to WGS84 conversion.

Finally, to activate the environment and execute arbitrary commands without ROS2, execute
```bash
cd geoscenarioserver
pixi shell
```
or with ROS2, execute
```bash
cd geoscenarioserver
pixi shell -e humble
```

#### Conda-forge and robostack (ROS) using micromamba

To automatically create a conda-forge environment called `gss` with the required packages, use the script `setup-conda-forge-env.bash`:
```bash
bash setup-conda-forge-env.bash --help

Create a conda-forge environment called gss for running GeoScenarioServer

Usage:
  $ bash setup_conda-forge_env.bash [-r|--ros2] [-t|--test-run] [-h|--help]
    -r|--ros2       install ROS2 humble and build tools into the environment 'gss'; build the ROS2 client
    -t|--test-run   start GeoScenarioServer within the environment 'gss'
    -h|--help       display usage instructions and exit
```


## Running

- run `gsserver -s scenarios/<geoscenario_file>` to start the Server.

```
usage: gsserver [-h] [-s [FILE ...]] [--verify_map FILE] [-q VERBOSE] [-n] [-m MAP_PATH] [-b BTREE_LOCATIONS] [-wi] [-wc] [-dp DASH_POS DASH_POS DASH_POS DASH_POS] [-d] [-fl] [-wt]

Starts the GeoScenario Server simulation

options:
  -h, --help            show this help message and exit
  -s [FILE ...], --scenario [FILE ...]
                        GeoScenario file. If no file is provided, the GSServer will load a scenario from code
  --verify_map FILE     Lanelet map file
  -q VERBOSE, --quiet VERBOSE
                        don't print messages to stdout
  -n, --no-dash         run without the dashboard
  -m MAP_PATH, --map-path MAP_PATH
                        Set the prefix to append to the value of the attribute `globalconfig->lanelet`
  -b BTREE_LOCATIONS, --btree-locations BTREE_LOCATIONS
                        Add higher priority locations to search for btrees by agent btypes
  -wi, --wait-for-input
                        Wait for the user to press [ENTER] to start the simulation
  -wc, --wait-for-client
                        Wait for a valid client state to start the simulation
  -dp DASH_POS DASH_POS DASH_POS DASH_POS, --dash-pos DASH_POS DASH_POS DASH_POS DASH_POS
                        Set the position of the dashboard window (x y width height)
  -d, --debug           Set the logging level to DEBUG instead of INFO
  -fl, --file-log       Log to $GSS_OUTPUTS/GSServer.log instead of stdout
  -wt, --write-trajectories
                        Write all agent trajectories to CSV files inside $GSS_OUTPUTS
```

GSServer creates various files on the folder `./outputs`, which can also be overridden using the environment variable `GSS_OUTPUTS`.

- GeoScenario files (2.0 required) must be placed inside *scenarios/*
- If a file is not given, you must provide a manual problem startup from code.
- LaneletMap files must be placed inside *scenarios/maps* (a map file is mandatory).
- Co-Simulator (Unreal or other) is optional.

## Loading multiple scenario files

- The `--scenario` option can take more than one `.osm` file as its arguments
- For example,
```bash
python3 GSServer.py --scenario scenarios/test_scenarios/gs_straight_obstacles.osm scenarios/test_scenarios/gs_straight_pedestrian.osm
```
- With the exception of `globalconfig` and `origin`, the elements from each scenario are loaded and combined at runtime
- The `globalconfig` and `origin` are used from the first `.osm` file that is specified (which is `gs_straight_obstacles.osm` in the example)
- Multiple scenarios can define vehicles and pedestrians with the same `vid`s and `pid`s
- If these scenarios are passed to the `--scenario` option, then an error will be reported
- All `vid` and `pid` conflicts must be resolved before running `GSServer.py`
- Scenarios can contain vehicles with no `vid` and pedestrians with no `pid`
- These vehicles and pedestrians will be auto-assigned `vid`s and `pid`s
- Auto-assigned `vid`s and `pids` will start from 1 and won't conflict with the other `vid`s and `pid`s

## Configuration:

- Check *SimConfig.py* for configuration options.
- Adjust FRAME_RATE based on hardware performance to avoid drift (Recommended 30Hz).
- Adjust PLANNER_RATE based on hardware performance and scenario requirements.
- Use SHOW_DASHBOARD = True for GUI. Adjust dashboard refresh rate according to performance.
- Simulations can only run in Real Time (so far).

## Building

To build a `.conda` package, execute
```bash
pixi build
```
To test the package in a fresh conda environment, execute
```bash
bash test/test_conda_package.bash
```

To build a `.whl` package, execute
```bash
pixi run build_wheel
```
To test the package in a fresh venv environment, execute
```bash
bash test/test_wheel_package.bash
```

## Co-Simulation:

### Native ROS2 server

To test the native ROS2 serever with a mock co-simulator, execute
```bash
bash test/test_ros2_server.bash [--fastest|--realtime|--2xrealtime]
```

### Via shared memory and ROS2 client

- Use the shared memory keys inside SimConfig to read/write the server shared memory blocks.
- We provide a GeoScenario Client for Unreal in `clients/unreal`.
- We provide a ROS2 Client for Humble in `ros2/geoscenario_client`.
To test the ROS2 client with a mock co-simulator, execute
```bash
bash test/test_ros2_client.bash
```

## Documentation:

GeoScenario documentation can be found [here](https://geoscenario2.readthedocs.io/en/latest/).

## Demo
GeoScenario Server running in High Fidelity Simulation with UE5 and Carla

[Youtube Video](https://youtu.be/Fk890JvgwWk?feature=shared)

## Questions?

rqueiroz@uwaterloo.ca
michal.antkiewicz@uwaterloo.ca