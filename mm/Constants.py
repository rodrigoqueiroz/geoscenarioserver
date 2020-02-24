#!/usr/bin/env python
#rqueiroz@gsd.uwaterloo.ca
# ---------------------------------------------
# CONFIGURABLE CONSTANTS
# --------------------------------------------
#TODO: get road attributes from a map

#ROAD
ROAD_W_STEP = 1                   #Road width sampling step [m]

#HV PARAMS
VEHICLE_RADIUS = 1.0                    #vehicle as a circle to simplify collision math

TARGET_VELOCITY = 30.0 / 3.6 #=8.3m/s      #HV target speed [m/s] 
TS_SAMP = 5.0 / 3.6  # ~1.38 m/s        #HV target speed sampling [m/s]
VELOCITY_STEP = 6
MAX_VELOCITY = 50 / 3.6
MIN_VELOCITY = 40 / 3.6

MAX_ACCEL = 2.0
EXPECTED_ACC_IN_ONE_SEC = 1     # m/s

MAX_JERK = 10               # maximum jerk [m/s/s/s]
EXPECTED_JERK_IN_ONE_SEC = 2    # m/s/s

#PLANNING TIMES
PLAN_TIME_STEP = 0.5    #general planning time step
VK_TIME = 4             #velocity keeping planning time 
VK_MIN_TIME = 2         #velocity keeping time variation (+ and -)
VK_MAX_TIME = 6

FL_TIME = 4             #following planning time
FL_MIN_TIME = 3         #following time variation (+ and -)
FL_MAX_TIME = 5

STOP_DISTANCE_STEP = 10

LC_TIME_VAR = 4
CI_TIME = 8
CI_TIME_VAR = 2 

#space sampling
N_SAMPLES = 10
SIGMA_S = [10.0, 2.0, 1.0] # s, s_d, s_dd
SIGMA_D = [0.5, 0.5, 0.5]
SIGMA_T = 2.0

#Struct:
    #plantime
    #time goal
    #min / max / time goal
    # s var
    # d var
    # n samples for pos s and d