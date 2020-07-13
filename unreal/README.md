## Co-Simulation with GeoScenarioModule for Unreal:

- The provided GeoScenario module includes *GSClient* actor to connect to GSServer and spawn mirror vehicles.
- The actor can be placed in the level, or spawned by the Engine request.
- Both simulations (Server and Client) can be started in any order.

### Unreal Client Integration:

Copy (or symlink) unreal/GeoScenarioModule into your unreal project source folder.
Copy (or symlink) unreal/GeoScenarioContent into your unreal project content folder.

Add GeoScenario Module to your *myprojectname.uproject* file:

```
"Modules":
[
	{
	"Name": "GeoScenarioModule",
	"Type": "Runtime",
	"LoadingPhase": "Default",
	"AdditionalDependencies": ["Engine"]
	}
]
```

Add GeoScenarioModule to the sTargetRules in files *Source/myprojectname.Target.cs* and *Source/myprojectnameEditor.Target.cs*:

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
the vehicle id must matche a remote id in the Server.


rqueiroz@gsd.uwaterloo.ca
d43sharm@uwaterloo.ca
