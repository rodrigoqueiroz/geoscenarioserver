#   GeoScenario Server
Includes: GeoScenario Parser, Checker, Maneuver Models, and Scenario Optimization modules.

### Dependencies (python3)
- numpy
- matplotlib
- python3-tk
- python3-sysv-ipc

## Maneuver Models (/mm)
- For Unreal Engine integration, use the module inside /unreal. This module must be added to the Unreal Project and the GeoScenario Actor classes added to the level.
- Use show_dashboard = True for independent view without Unreal
- Simulations run in Real Time. Adjust fame rate based on performance to avoid drift (Recommended 30Hz).

### Running
- run `python3 mm/Simulator.py` first, THEN the Unreal simulator (optional)


rqueiroz@gsd.uwaterloo.ca
d43sharm@edu.uwaterloo.ca
