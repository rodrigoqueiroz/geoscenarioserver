#   GeoScenario Server
Includes: GeoScenario Parser, Checker, Sim Vehicle Planner with Behavior Trees and Maneuver Models.

## Dependencies

### Apt packages

- python3.8
- python3.8-dev
- python3.8-venv
- python3-tk
- python3-pip

### Python packages

- numpy
- glog
- matplotlib
- scipy
- [py_trees](https://github.com/splintered-reality/py_trees)
- tk
- sysv-ipc
- antlr4-python3-runtime
- antlr-denter

### Source packages

- Lanelet2 (submodule)

To automatically install the dependencies, execute
```
bash scripts/install_dependencies.bash
```

## Running
- run `python3.8 GSServer.py -s scenarios/<geoscenario_file>` to start the Server.
```
optional arguments:
  -h, --help            show this help message and exit
  -s FILE, --scenario FILE
                        GeoScenario file
  -q VERBOSE, --quiet VERBOSE
                        don't print messages to stdout
  -m, --map-path 
                        Set the prefix to append to the value of the attribute `globalconfig->lanelet`
                        e.g. --map-path $HOME/anm_unreal_test_suite/maps
  -b, --btree-locations 
                        Add higher priority locations to search for btrees by agent btypes
                        e.g. --btree-locations $HOME/anm_unreal_test_suite/btrees
```

- GeoScenario files (2.0 required) must be placed inside *scenarios/*
- If a file is not given, you must provide a manual problem startup from code.
- LaneletMap files must be placed inside *scenarios/maps* (a map file is mandatory).
- Co-Simulator (Unreal or other) is optional.

## Configuration:

- Check *SimConfig.py* for configuration options.
- Adjust FRAME_RATE based on hardware performance to avoid drift (Recommended 30Hz).
- Adjust PLANNER_RATE based on hardware performance and what scenario requirements.
- Use SHOW_DASHBOARD = True for GUI. Adjust dashboard refresh rate according to performance.
- Simulations can only run in Real Time (so far).

## Co-Simulation:

- Use the shared memory keys inside SimConfig to read/write the server shared memory blocks.
- We provide a GeoScenario Client for Unreal in */unreal*.


rqueiroz@gsd.uwaterloo.ca
d43sharm@uwaterloo.ca
