#   GeoScenario Server
Includes: GeoScenario Parser, Checker, Sim Vehicle Planner with Behavior Trees and Maneuver Models.

## Dependencies:
- python 3.8
- numpy
- matplotlib
- python3-tk
- python3-sysv-ipc
- [py_trees](https://github.com/splintered-reality/py_trees)

#### Lanelet Dependencies:
- [lanelet2](https://github.com/yuzhangbit/lanelet2_standalone)
- Boost 1.58 (does not compile with Boost 1.6.5 or newer)
- eigen3
- pugixml
- boost-python3
- geographiclib


## Running
- run `python GSServer.py -s scenarios/<geoscenario_file>` to start the Server.
```
optional arguments:
  -h, --help            show this help message and exit
  -s FILE, --scenario FILE
                        GeoScenario file
  -q VERBOSE, --quiet VERBOSE
                        don't print messages to stdout
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
