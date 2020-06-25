#!/usr/bin/env python
#rqueiroz@gsd.uwaterloo.ca
# ---------------------------------------------
# CONFIGURABLE CONSTANTS
# --------------------------------------------

#SIM CONFIG
TIMEOUT = 20               #timeout in [s]
TRAFFIC_RATE = 30             #Global tick rate

#Planner Process (BTree and MM)
PLANNER_RATE = 5

#DASH CONFIG
SHOW_DASHBOARD = True      	#plot vehicles and trajectories. Optional when running with Ureal engine.
DASH_RATE = 20
PLOT_VID = 1               	#Vehicle to center the main plot arouund. Make sure there existis a vehicle with this id
FFPLOT_ASPECT = False  		#Frenet frame plot: keep S and D aspect (same scale)
FFPLOT_LENGTH = 100			#Frenet frame plot: road length (s) in meters
CPLOT_SIZE = 80				#Cartesian plot: road length in meters (shorter=better performance)

VEH_STAT_CHART = False      #(!HEAVY) Vehicle Stat Chart. Limited to PLOT_VID
VEH_TRAJ_CHART = False      #(!HEAVY) Show detailed trajectory chart, with Lat and Long Velocity and Acceleraton. Limited to PLOT_VID

#
VEH_COLLISION = False      	#If true, collision between vehicles with be considered during planning.
OBJ_COLLISION = False      	#If true, collision between vehicles and static objects on the road with be considered during planning.

COLL_TYPE_RADIUS = True     #vehicle is computed as a circle to simplify collision math and lane boundary checks
VEHICLE_RADIUS = 1.0        #vehicle radius

COLL_TYPE_CORNERS = False       #vehicle is computed as 4 circles on the corners
COLLISION_CORNER_RADIUS = 0.2   #radius for each corner

#ROAD
NUM_SAMPLING_D = 3         	#number of sampling points on the lane width for lateral planning


#Client (Unreal or similar)
CLIENT_METER_UNIT = 100    	#Client unit (Server uses [m], Unreal client uses [cm])

