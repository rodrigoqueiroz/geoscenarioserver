#!/usr/bin/env python3
#rqueiroz@uwaterloo.ca
#d43sharm@uwaterloo.ca
# --------------------------------------------
# SimConfig
# General configuration settings
# --------------------------------------------

from dataclasses import dataclass, field
from typing import Dict
import os
import math

ROOT_DIR = os.path.dirname(os.path.realpath(__file__))

#Sim Config
TIMEOUT = 30                #default timeout in [s] if not defined by scenario
TRAFFIC_RATE = 40           #global tick rate
WAIT_FOR_INPUT = False      #wait for user input before starting simulation

#Dash Config
SHOW_DASHBOARD = True       #show dash with plots, vehicles and trajectories. Optional.
DASH_RATE = 10              #dash tick rate. Max is traffic rate.
PLOT_VID = 1                #vehicle to center the main plot around, if not defined by the scenario.
                            #Make sure there exists a vehicle with this id
#Global Map
SHOW_MPLOT = True           #whether to show the global map cartesian plot
MPLOT_SIZE = 80			    #map plot: road length in meters (shorter=better performance)
#Frenet Map
SHOW_FFPLOT = True          #whether to show the frenet frame plot
SHOW_OCCUPANCY = True       #whether to show road and intersection occupancy inside the frenet plot
FFPLOT_ASPECT = True  		#frenet frame plot: keep S and D aspect ratio (same scale)
FFPLOT_LENGTH = 60			#frenet frame plot: road length (s) in meters
FFPLOT_LITE = False         #frenet frame plot: if true, plots a simplified version with onlye self vehicle. If false, plots all vehicles, trajectories and candidates
#Cartesian
SHOW_CPLOT = True           #whether to show the cartesian plot
CPLOT_SIZE = 100            #cartesian plot: road length in meters (shorter=better performance)
REFERENCE_PATH = True       #reference path indicating the frenet frame for the vehicle
SHOW_VEHICLE_SHAPE = True   #vehicle plot with rectangle shape.
SHOW_VEHICLE_RADIUS = False #vehicle plot with radius.

#Vehicle trajectory
VEH_TRAJ_CHART = False      #(!HEAVY, not stable) Show detailed trajectory chart, with Lat and Long Velocity and Acceleraton. Limited to PLOT_VID

#BTree
SHOW_BTREE = True           #whether to show the current behaviour tree
SHOW_MCONFIG = True         #whether to show the last selected maneuver config together with tree
GENERATE_GRAPH_TREE = True  #whether to generate a behavior tree graph plot inside GSS_OUTPUTS

# trajectory plots
SHOW_TRAJ = False

#Collision
COLL_TYPE_RADIUS = True     #vehicle is computed as a circle to simplify collision math and lane boundary checks

#Standard vehicle dimensions
VEHICLE_RADIUS = 0.9        #vehicle radius
VEHICLE_LENGTH = 4.5       #vehicle length in [m]
VEHICLE_WIDTH = 1.8         #vehicle width in [m]

# Source NCAP: https://cdn.euroncap.com/media/58226/euro-ncap-aeb-vru-test-protocol-v303.pdf
# Pedestrian dimensions (width: 0.5 m, length: 0.6 m) approximated by a circle with radius 0.27 m
PEDESTRIAN_RADIUS = 0.27
PEDESTRIAN_LENGTH = 0.6
PEDESTRIAN_WIDTH = 0.5

#Planning
PLANNER_RATE = 5                 #Planner tick rate
PLANNING_TIME = 0.2              #[s] Must be <= 1/PLANNTER_RATE (we recommend 0.100 for scenarios with <4 vehicles)
USE_FIXED_PLANNING_TIME = True   #True: the plan will target PLANNING_TIME. False, the planner will vary between PLANNING_TIME and max time (1/PLANNTER_RATE)
POINTS_PER_METER = 3.0           #The number of points per meter to be used along the vehicle's reference path
                                 #Note that the value that is used may be slightly different

#Debugging and Log
PLOT_VEHICLE_ROUTES = False    #If True, will open figures for each of a vehicle's global paths
                               #Each figure will contain the map (black), the route (red), and the global path (blue)
                               #Figures are opened for each vehicle in the scenario
                               #This should only be set when you want to see these figures
                               #The program will crash after showing figures for all vehicles (an XIO error)
LOG_PERFORMANCE = False
MAX_NVEHICLES = math.inf       #Limit max number of active vehicles (math.inf if no limit)
EVALUATION_MODE = False
WRITE_TRAJECTORIES = False     #If True, all vehicle trajectories will be saved inside eval/ as csv files

#Client (Unreal or similar)

#Carla
CARLA_COSIMULATION = False
CARLA_SERVER = 'localhost'
CARLA_PORT = 2000

#Shared Memory
WAIT_FOR_CLIENT = False     #Hold Simulation start until a valid state is sent from client
CLIENT_SHM = True           #If True, server will create shared memory space to exchange data with client.
SHM_KEY = 123456
SEM_KEY = 346565
CS_SHM_KEY = 333943
CS_SEM_KEY = 933433
SHM_SIZE = 2048

# osm file merging config
# list of gs tags that must be unique per scenario
UNIQUE_GS_TAGS_PER_SCENARIO = ['origin', 'globalconfig']

@dataclass
class SimConfig:
    pedestrian_lanelet_routes:Dict = field(default_factory=dict)
    pedestrian_goal_points:Dict = field(default_factory=dict)
    scenario_name:str = "Unnamed scenario"
    map_name:str = "Unknown map"
    timeout:int = TIMEOUT
    traffic_rate:int = TRAFFIC_RATE
    planner_rate:int = PLANNER_RATE
    planning_time:float = PLANNING_TIME
    use_fixed_planning_time:bool = USE_FIXED_PLANNING_TIME
    plot_vid:int = PLOT_VID
    show_dashboard:bool = SHOW_DASHBOARD
    wait_for_input:bool = WAIT_FOR_INPUT
    wait_for_client:bool = WAIT_FOR_CLIENT
