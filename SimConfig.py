#!/usr/bin/env python3
#rqueiroz@guwaterloo.ca
#d43sharm@uwaterloo.ca
# --------------------------------------------
# SimConfig
# Class to hold configuration settings of the scenario + config constants
# --------------------------------------------

from dataclasses import dataclass, field
from typing import Dict
import os

ROOT_DIR = os.path.dirname(os.path.realpath(__file__))

#Sim Config
TIMEOUT = 30                #default timeout in [s] if not defined by scenario
TRAFFIC_RATE = 30           #global tick rate
WAIT_FOR_INPUT = False      #wait for user input before starting simulation

#Dash Config
SHOW_DASHBOARD = True      	#show dash with plots, vehicles and trajectories. Optional.
DASH_RATE = 30              #dash tick rate. Max is traffic rate.
PLOT_VID = 1               	#vehicle to center the main plot around, if not defined by scenario.
                            #Make sure there exists a vehicle with this id
#Global Map
SHOW_MPLOT = True           #whether to show the global map cartesian plot
MPLOT_SIZE = 100			#map plot: road length in meters (shorter=better performance)
#Frenet Map
SHOW_FFPLOT = True          #whether to show the frenet frame plot
FFPLOT_ASPECT = False  		#frenet frame plot: keep S and D aspect ratio (same scale)
FFPLOT_LENGTH = 60			#frenet frame plot: road length (s) in meters
FFPLOT_LITE = False         #frenet frame plot: if true, plots a simplified version with onlye self vehicle. If false, plots all vehicles, trajectories and candidates
#Cartesian
SHOW_CPLOT = True           #whether to show the cartesian plot
CPLOT_SIZE = 40			    #cartesian plot: road length in meters (shorter=better performance)
REFERENCE_PATH = True       #reference path indicating the frenet frame for the vehicle
#Vehicle trajectory
VEH_TRAJ_CHART = False      #(!HEAVY) Show detailed trajectory chart, with Lat and Long Velocity and Acceleraton. Limited to PLOT_VID
#BTree
SHOW_BTREE = True           #whether to show the current behaviour tree
# trajectory plots
SHOW_TRAJ = False

#Collision
COLL_TYPE_RADIUS = True     #vehicle is computed as a circle to simplify collision math and lane boundary checks
VEHICLE_RADIUS = 1.0        #vehicle radius
COLL_TYPE_CORNERS = False       #vehicle is computed as 4 circles on the corners
COLLISION_CORNER_RADIUS = 0.2   #radius for each corner

#Planning
PLANNER_RATE = 5            #Planner tick rate
#Collision
#collision and proximity cost have a great impacto on performance. 
#For example, if the scenario does not require object avoidance, you can turn it off.
VEH_COLLISION = False      	#If true, collision between vehicles with be considered during planning.
PED_COLLISION = False       #If true, collision between vehicles and pedestrians on the road with be considered during planning.
OBJ_COLLISION = False      	#If true, collision between vehicles and static objects on the road with be considered during planning.
TRAJECTORY_SPLIT = 10       #In how many parts a trajectory is split for verification. Higher(100) = better precision, but impacts performance.


#Evaluation
WRITE_TRAJECTORIES = False     #If True, all vehicle trajectories will be saved inside eval/ as csv files

#Client (Unreal or similar)
CLIENT_METER_UNIT = 100    	#Client unit (Server uses [m], Unreal client uses [cm])

#Shared Memory
CLIENT_SHM = False          #If True, server will create shared memory space to exchange data with client.
SHM_KEY = 123456
SEM_KEY = 346565
CS_SHM_KEY = 333943
CS_SEM_KEY = 933433
SHM_SIZE = 2048

@dataclass
class SimConfig:
    lanelet_routes:Dict = field(default_factory=dict)
    goal_points:Dict = field(default_factory=dict)
    pedestrian_goal_points:Dict = field(default_factory=dict)
    scenario_name:str = "Unamed scenario"
    map_name:str = "Unknown map"
    timeout:int = TIMEOUT
    traffic_rate:int = TRAFFIC_RATE
    plot_vid:int = PLOT_VID
