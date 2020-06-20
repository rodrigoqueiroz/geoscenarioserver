#   GeoScenario Server
Includes: GeoScenario Parser, Checker, Sim Vehicle Planner with Maneuver Models, and Scenario Optimization modules.

### Dependencies (python3)
- python 3.8
- numpy
- matplotlib
- python3-tk
- python3-sysv-ipc
- [lanelet2](https://github.com/yuzhangbit/lanelet2_standalone)

## Simulated Vehicle Planner and Maneuver Models (/sv)
- Check util/Constants for configuration options. Use SHOW_DASHBOARD = True for independent view (no Unreal, but affect performance).
- Simulations run in Real Time. Adjust FRAME_RATE based on hardware performance to avoid drift (Recommended 30Hz).

## Running
- run `python3 Simulator.py` to start he Server
- Unreal simulator is optional, and can be started in any order.


## Unreal Integration
Copy (or symlink) unreal/GeoScenarioModule into your unreal project source folder.
Copy (or symlink) unreal/GeoScenarioContent into your unreal project content folder.

Add GeoScenario Module to your *myprojectname.uproject* file:

```
"Modules": [
    {
    "Name": "GeoScenarioModule",
    "Type": "Runtime",
    "LoadingPhase": "Default",
    "AdditionalDependencies": [
        "Engine"
        ]
	}
```

Add GeoScenarioModule to the TargetRules in files *Source/myprojectname.Target.cs* and *Source/myprojectnameEditor.Target.cs*:

```
ExtraModuleNames.AddRange( new string[] {"GeoScenarioModule" } );
```
Rebuild your Unreal Project

### How to run

Add GSClient (actor) to your level. 
When the level starts, the client will attempt to connect to GeoScenario Server.
When the connection is estabilished and the Server starts publishing Vehicles states, the client will spawn Simulated Vehicles and sync with Server.
In order to sync vehicles spawned and controlled by the client (e.g., Ego), add the following tags to the respective Actor instance (or Pawn) and GSClient will find them. 

```
gsvehicle
vid:x
```
the vehicle id must match a remote id in the Server.


rqueiroz@gsd.uwaterloo.ca

d43sharm@uwaterloo.ca