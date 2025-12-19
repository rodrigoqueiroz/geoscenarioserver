# Scenario suite

Scenario suite is a convention for organizing scenarios into base scenario and scenario parts.
The suite is supported by convenience scripts.

The scenario suite contains the following folders:
```
scenario_suite/
├── geoscenarioserver/        # symlink to or submodule of [geoscenarioserver](https://github.com/rodrigoqueiroz/geoscenarioserver/)
├── maps/                     # map files used by scenarios (or symlinks)
├── scenarios/                # scenario definitions
│   ├── <scenario_name>/      # base scenario folder
│   │   ├── <scenario_name>.osm   # base scenario file
│   │   └── parts/                # optional scenario parts folder
│   │       ├── <part_name>.osm    # scenario part file
│   │       └── ...
├── logs/                     # scenario execution logs
├── scripts/                  # scenario launch scripts
└── setup.bash                # script to make the command 'slaunch' available
```

## Installation

Running natively on any linux OS and `bash` shell

Execute
```bash
$ cd scenario_suite
$ source setup.bash
$ slaunch
```
Follow the instructions to install `pixi`.
The script lists the available scenarios.

## Usage

1. source the environment (provides the alias `slaunch` and allows launching scenarios from any working directory):
```bash
$ cd scenario_suite
$ source setup.bash
```

2. Launch a scenario called `<scenario_name>` and, optionally, a number of scenario parts `<part_name>.osm*`.
A list of `[<gss_options>*]`, such as, `--no-dash`, can be optionally provided as well.
Part names and GSS options can be intermixed.
By default, the GeoScenarioServer is launched in standalone mode but the native ROS2 node can be launched instead by providing the `--ros` flag.

```bash
$ slaunch

Usage: slaunch <scenario_name> [<part_name>.osm*] [--ros] [<gss_options>*]

Launches the specified scenario with the GeoScenarioServer traffic simulator
located at /.../scenario_suite/geoscenarioserver.

Arguments:
     <scenario_name>      name of the folder 'scenarios/<scenario_name>' (mandatory)
     [<part_name>.osm*]   names of the files in the folder 'scenarios/<scenario_name>/parts' (optional list)
     [--ros]              launch ROS2 node geoscenario_server (launch standalone by default)
     [--mock-co-sim <real_time_factor>]
                          launch mock_co_simulator with delta_time=0.025 and the given real_time_factor (only valid with --ros)
                          real_time_factor: 0: max speed, (0..1): faster than real time, 1: real time, >1 = slower than real time
     [<gss_options>*]     additional options for the GeoScenarioServer (optional list):
                          --no-dash --wait-for-input --wait-for-client --dash-pos --debug --file-log --write-trajectories --origin-from-vid
...
```
for example, launch a standalone simulation
```bash
slaunch colliding_pedestrians \         # base secenario
        occluding-pv30-pv40.osm \       # 3 parts for traffic agents
        right-pp2.osm \
        left-pp3.osm \
        vut_sdv1-forward.osm \          # vehicle under test (VUT) SDV driving forward
        --dash-pos 0 0 960 1080  \      # GSS option
        front-pp1.osm \                 # another part
        --wait-for-input \              # other GSS options
        --file-log \
        --write-trajectories
```
for example, launch the native ROS2 server with a co-simulator
```bash
slaunch colliding_pedestrians \         # base secenario
        occluding-pv30-pv40.osm \       # 3 parts for traffic agents
        right-pp2.osm \
        left-pp3.osm \
        vut_sdv1-reverse.osm \          # vehicle under test (VUT) SDV driving reverse
        front-pp1.osm \                 # another part
        --ros \                         # start the native ROS2 server
        --no-dash                       # other GSS options
        --origin-from-vid 1             # override origin from the starting position of vehicle with ID 1
```

Each scenario may optionally contain a subfolder `parts`, which contains the available scenario fragments. 
For example ('VUT': vehicle under test):, 
```bash
scenarios/colliding_pedestrians/
├── colliding_pedestrians.osm           # base scenario with origin, map, and any common elements
└── parts                               # scenario fragments
    ├── front-sp1.osm                   # simulated pedestrian p1 walking in front of VUT
    ├── jaywalker-pp4-const.osm         # path pedestrian jaywalker p4 with constant speed
    ├── jaywalker-pp5-profile.osm       # path pedestrian jaywalker p5 with speed profile
    ├── left-sp3.osm                    # simulated pedestrian p3 entering from the left of VUT
    ├── occluding-pv10-pv20.osm         # occluding vehicles pv10 and pv20 after the crosswalk
    ├── occluding-pv30-pv40.osm         # occluding vehicles pv30 and pv40 before the crosswalk
    ├── right-sp2.osm                   # simulated pedestrian p2 entering from the right of VUT
    ├── vut_ev1-forward.osm             # VUT external vehicle in the forward start position
    ├── vut_ev1-reverse.osm             # VUT external vehicle in the reverse start position
    ├── vut_sdv1-forward.osm            # VUT SDV driving forward
    ├── vut_sdv1-forward-stop.osm       # VUT SDV driving forward and stopping before crosswalk
    └── vut_sdv1-reverse.osm            # VUT SDV driving reverse
```

NOTE: scenario fragments cannot be run individually without a base scenario because they are missing the required elements `origin` and `globalconfig`.

3. Review the scenario execution outputs

Every scenario execution creates a folder with various log files in 
```
scenario_suite/logs/<date>-<time>/
```
For example, 
```
scenario_suite/logs/2025-12-18-172646/
├── gss_output.log       # console output
├── launch_command.log   # command used to launch the simulation 
├── launch_params.yaml   # (ROS2 only) launch command parameters
├── trajectory_v<id>.csv # recorded trajectory of vehicle with ID <id> (if --write-trajectories option used)
└── violations.json      # report of collisions, timeouts, and other events
```

For ROS2, the `launch_params.yaml` is used to provide values of parameters to `ros2 run` command.

## Defining scenarios

Refer to [https://geoscenario2.readthedocs.io](https://geoscenario2.readthedocs.io).

### JOSM editor

Install using `snap install josm` or from [Ubuntu/Debian repository](https://josm.openstreetmap.de/wiki/Download#Ubuntu).

Configure according to [instructions](https://geoscenario2.readthedocs.io/en/latest/Tools/#geoscenario-editor-custom-josm).
