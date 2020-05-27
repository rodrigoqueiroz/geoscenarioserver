#!/usr/bin/env python
#rqueiroz@gsd.uwaterloo.ca
# ---------------------------------------------
# CONFIGURABLE CONSTANTS
# --------------------------------------------

#SIM CONFIG
TIMEOUT = 20               #timeout in [s]
FRAME_RATE = 30            #Global tick rate
SHOW_DASHBOARD = True      #plot vehicles and trajectories. Optional when running with Ureal engine.
PLOT_VID = 1                #Vehicle to center the plot arouund. Make sure there is a vehicle with this id
CHART_ASPECT_EQUAL = False
#SIM CONFIG
VEH_COLLISION = True        #If true, collision between vehicles with be considered during planning.
OBJ_COLLISION = False       #If true, collision between vehicles and static objects on the road with be considered during planning.

COLL_TYPE_RADIUS = True     #vehicle is computed as a circle to simplify collision math and lane boundary checks
VEHICLE_RADIUS = 1.0        #vehicle radius

COLL_TYPE_CORNERS = False       #vehicle is computed as 4 circles on the corners
COLLISION_CORNER_RADIUS = 0.2   #radius for each corner

#ROAD
NUM_SAMPLING_D = 3            #number of sampling points on the lane width for lateral planning

