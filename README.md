#   GeoScenario Server

Includes: GeoScenario Parser, Checker, Sim Vehicle Planner with Behavior Trees and Maneuver Models.

## Dependencies

- Linux, macOS, or Windows 10/11 + WSL2
- Python >= 3.8

GeoScenario Server can run natively on Linux, within a [conda-forge](https://conda-forge.org/) environment, or on WSL2.

### Deb packages for Linux native

Tested on native Ubuntu 20.04, 22.04, 24.04, and within Windows 10 WSL2.

- python3
- python3-dev
- python3-tk
- python3-pip
- python3-pil
- python3-pil.imagetk

### Python packages

- antlr4-python3-runtime==4.9.3 (later versions cause a parsing error `Exception: Could not deserialize ATN with version (expected 4).`)
- antlr-denter
- glog
- lanelet2
- matplotlib
- numpy
- scipy
- [py_trees==0.7.6](https://github.com/splintered-reality/py_trees)
- sysv-ipc
- tk

To automatically install the dependencies for linux native, execute

```
bash scripts/install_dependencies.bash
```

Alternatively, to automatically create a conda-forge environment called `gss` with the required packages, use the script `setup-conda-forge-env.bash`:
```
$ bash setup-conda-forge-env.bash --help

Create a conda-forge environment called gss for running GeoScenarioServer

Usage:
  $ bash setup_conda-forge_env.bash [-r|--ros2] [-t|--test-run] [-h|--help]
    -r|--ros2       install ROS2 humble and build tools into the environment 'gss'; build the ROS2 client
    -t|--test-run   start GeoScenarioServer within the environment 'gss'
    -h|--help       display usage instructions and exit
```

## Running

- run `python3 GSServer.py -s scenarios/<geoscenario_file>` to start the Server.

```
usage: GSServer.py [-h] [-s [FILE ...]] [--verify_map FILE] [-q VERBOSE] [-n] [-m MAP_PATH] [-b BTREE_LOCATIONS] [-wi] [-wc]

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
```

- GeoScenario files (2.0 required) must be placed inside *scenarios/*
- If a file is not given, you must provide a manual problem startup from code.
- LaneletMap files must be placed inside *scenarios/maps* (a map file is mandatory).
- Co-Simulator (Unreal or other) is optional.

## Loading multiple scenario files

- The `--scenario` option can take more than one `.osm` file as its arguments
- For example,
```
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
- Adjust PLANNER_RATE based on hardware performance and what scenario requirements.
- Use SHOW_DASHBOARD = True for GUI. Adjust dashboard refresh rate according to performance.
- Simulations can only run in Real Time (so far).

## Co-Simulation:

- Use the shared memory keys inside SimConfig to read/write the server shared memory blocks.
- We provide a GeoScenario Client for Unreal in */unreal*.

## Documentation:

GeoScenario documentation can be found [here](https://geoscenario2.readthedocs.io/en/latest/).

## Demo
GeoScenario Server running in High Fidelity Simulation with UE5 and Carla

[Youtube Video](https://youtu.be/Fk890JvgwWk?feature=shared)

## Questions?
rqueiroz@uwaterloo.ca
