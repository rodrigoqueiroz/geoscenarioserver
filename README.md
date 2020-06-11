#   GeoScenario Server
Includes: GeoScenario Parser, Checker, Sim Vehicle Planner with Maneuver Models, and Scenario Optimization modules.

### Dependencies (python3)
- python 3.8
- numpy
- matplotlib
- python3-tk
- python3-sysv-ipc

## Simulated Vehicle Planner and Maneuver Models (/sv)
- For Unreal Engine integration, use the module inside /unreal. This module must be added to the Unreal Project and the GeoScenario Actor classes added to the level.
- Check util/Constants for configuration options. Use SHOW_DASHBOARD = True for independent view (no Unreal).
- Simulations run in Real Time. Adjust FRAME_RATE based on hardware performance to avoid drift (Recommended 30Hz).

### Running
- run `python3 Simulator.py` first, THEN the Unreal simulator (optional)


rqueiroz@gsd.uwaterloo.ca
d43sharm@uwaterloo.ca
