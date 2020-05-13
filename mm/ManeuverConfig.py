#!/usr/bin/env python
#rqueiroz@gsd.uwaterloo.ca
# ---------------------------------------------
# CONFIG Data Classes and Constants for Maneuvers
# --------------------------------------------
from dataclasses import dataclass


#TARGET_VELOCITY = 30.0 / 3.6 #=8.3m/s      #HV target speed [m/s] 
#TS_SAMP = 5.0 / 3.6  # ~1.38 m/s        #HV target speed sampling [m/s]
#VELOCITY_STEP = 6
MAX_VELOCITY = 30 / 3.6                 #11 m/s
MIN_VELOCITY = 28 / 3.6
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

EXPECTED_OFFSET_PER_SEC = 20.0

#maneuvers
M_VELKEEPING = 1
M_LANECHANGE = 2
M_CUTIN = 3
M_FOLLOW = 4
M_STOP = 5

#todo: add all config structs and default values (CONSTAMTS) to a separate file

@dataclass
class LaneConfig():
    def __init__(self, max_velocity, left_boundary, right_boundary):
        self.max_velocity = max_velocity
        self.left_boundary = left_boundary
        self.right_boundary = right_boundary

@dataclass
class MStopConfig():
    def __init__(self, time_range, deceleration_range = None, min_distance = 30):
        #self.distance_range = distance_range
        #self.stop_pos = stop_pos
        self.time_range = time_range
        self.deceleration_range = deceleration_range
        self.min_distance = min_distance

@dataclass        
class MVelKeepingConfig():
    def __init__(self, vel_range, time_range ):
        self.vel_range = vel_range
        self.time_range = time_range

@dataclass
class MFollowConfig():
    def __init__(self, time_range,time_gap, distance):
        self.time_range = time_range
        self.time_gap = time_gap
        self.distance = distance
        self.min_distance  = 50.0 #min distance to collision [meters]
        self.min_ttc = 10.0 #min time to collision [seconds]

@dataclass
class MLaneChangeConfig():
    def __init__(self, time_range, time_gap, distance):
        self.time_range = time_range
        self.time_gap = time_gap
        self.distance = distance

@dataclass
class MCutInConfig():
    def __init__(self, time_range, time_gap, distance, delta):
        self.time_range = time_range
        self.time_gap = time_gap
        self.distance = distance
        self.delta = delta
