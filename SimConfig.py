#!/usr/bin/env python3
# rqueiroz@uwaterloo.ca
# d43sharm@uwaterloo.ca
# --------------------------------------------
# SimConfig
# General configuration settings
# --------------------------------------------

from dataclasses import dataclass, field
from typing import Dict
import os
import math

ROOT_DIR = os.path.dirname(os.path.realpath(__file__))

# --- Sim Config
# default timeout in [s] if not defined by scenario
TIMEOUT = 30
# global tick rate
TRAFFIC_RATE = 30
# wait for user input before starting simulation
WAIT_FOR_INPUT = False

# --- Dash Config
# show dash with plots, vehicles and trajectories. Optional.
SHOW_DASHBOARD = True
# dash tick rate. Max is traffic rate.
DASH_RATE = 30
# dash tick rate. Max is traffic rate.
DASH_RATE_FALLBACK = 10
# vehicle to center the main plot around, if not defined by the scenario.
# Make sure there exists a vehicle with this id
PLOT_VID = 1

# Make individual local catesian plots for at most 4 vehicles
SHOW_INDIVIDUAL_VEHICLE_PLOTS = True

# --- Global Map
# whether to show the global map cartesian plot
SHOW_MPLOT = True
# map plot: road length in meters (shorter=better performance)
MPLOT_SIZE = 120

# --- Frenet Map
# whether to show the frenet frame plot
SHOW_FFPLOT = True
# frenet frame plot: keep S and D aspect ratio (same scale)
FFPLOT_ASPECT = False
# frenet frame plot: road length (s) in meters
FFPLOT_LENGTH = 60
# frenet frame plot: if true, plots a simplified version with onlye self vehicle. If false, plots all vehicles, trajectories and candidates
FFPLOT_LITE = False

# --- Local Map
# whether to show the cartesian plot
SHOW_CPLOT = False
# cartesian plot: road length in meters (shorter=better performance)
CPLOT_SIZE = 60
# reference path indicating the frenet frame for the vehicle
REFERENCE_PATH = True
# vehicle plot with rectangle shape.
SHOW_VEHICLE_SHAPE = True
# vehicle plot with radius.
SHOW_VEHICLE_RADIUS = False

# --- Vehicle trajectory
# (!HEAVY, not stable) Show detailed trajectory chart, with Lat and Long Velocity and Acceleraton. Limited to PLOT_VID
VEH_TRAJ_CHART = False

# --- BTree
# whether to show the current behaviour tree
SHOW_BTREE = True
# whether to show the last selected maneuver config together with tree
SHOW_MCONFIG = True
# whether to generate a behavior tree graph plot inside /log
GENERATE_GRAPH_TREE = True

# --- Trajectory Plots
SHOW_TRAJ = False

# --- Collision
# vehicle is computed as a circle to simplify collision math and lane boundary checks
COLL_TYPE_RADIUS = True

# --- Standard vehicle dimensions
# vehicle radius
VEHICLE_RADIUS = 1.0
# vehicle length in [m]
VEHICLE_LENGTH = 4.5
# vehicle width in [m]
VEHICLE_WIDTH = 1.8

# --- Planning
# Planner tick rate
PLANNER_RATE = 3
# [s] Must be <= 1/PLANNTER_RATE (we recommend 0.100 for scenarios with <4 vehicles)
PLANNING_TIME = 0.33
# True: the plan will target PLANNING_TIME. False, the planner will vary between PLANNING_TIME and max time (1/PLANNTER_RATE)
USE_FIXED_PLANNING_TIME = True
# The number of points per meter to be used along the vehicle's reference path
# Note that the value that is used may be slightly different
POINTS_PER_METER = 3.0

# --- Debugging and Log
# If True, will open figures for each of a vehicle's global paths
# Each figure will contain the map (black), the route (red), and the global path (blue)
# Figures are opened for each vehicle in the scenario
# This should only be set when you want to see these figures
# The program will crash after showing figures for all vehicles (an XIO error)
PLOT_VEHICLE_ROUTES = False
LOG_PERFORMANCE = False
# Limit max number of active vehicles (math.inf if no limit)
MAX_NVEHICLES = math.inf
EVALUATION_MODE = False
# If True, all vehicle trajectories will be saved inside eval/ as csv files
WRITE_TRAJECTORIES = False

# --- Client (Unreal or similar)
# Client unit (Server uses [m], Unreal client uses [cm], Carla uses [m])
CLIENT_METER_UNIT = 100

# --- Carla
CARLA_COSIMULATION = False
CARLA_SERVER = 'localhost'
CARLA_PORT = 2000

# Shared Memory
# Hold Simulation start until a valid state is sent from client
WAIT_FOR_CLIENT = False
# If True, server will create shared memory space to exchange data with client.
CLIENT_SHM = False
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
    pedestrian_lanelet_routes: Dict = field(default_factory=dict)
    pedestrian_goal_points: Dict = field(default_factory=dict)
    scenario_name: str = "Unamed scenario"
    map_name: str = "Unknown map"
    timeout: int = TIMEOUT
    traffic_rate: int = TRAFFIC_RATE
    plot_vid: int = PLOT_VID
    show_dashboard: bool = SHOW_DASHBOARD
