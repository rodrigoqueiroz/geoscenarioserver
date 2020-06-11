#!/usr/bin/env python
#rqueiroz@gsd.uwaterloo.ca
# ---------------------------------------------
# CONFIG Data Classes and Constants for Maneuvers
# --------------------------------------------
from __future__ import annotations  #Must be first Include. Will be standard in Python4
from dataclasses import dataclass
import random
import numpy as np

#Micro maneuvers
M_VELKEEP = 1
M_FOLLOW = 2
M_LANESWERVE = 3
M_CUTIN = 4
M_STOP = 5

#Feasibility Constants

#Cost Constants
EXPECTED_OFFSET_PER_SEC = 20.0
EXPECTED_ACC_PER_SEC = 1         # [m/s]
EXPECTED_JERK_PER_SEC = 2        # [m/s/s]
MAX_JERK = 10                    # maximum jerk [m/s/s/s]




@dataclass
class LaneConfig:                   
    #Lane Config in Frenet Frame
    #Left and Right lanes exist only if traversal is possible
    id:int = 0
    max_velocity:float = 30         # [m/s]
    left_bound:float = 4            #d
    right_bound:float = 0           #d
    _left_lane: LaneConfig = None
    _right_lane: LaneConfig = None
    
    def set_left_lane(self,lane):
        self._left_lane = lane
        lane._right_lane = self
    
    def set_right_lane(self,lane):
        self._right_lane = lane
        lane._left_lane = self
    
    def get_neighbour(self,l_id):
        if (self._right_lane):
            if (self._right_lane.id == l_id):
                return self._right_lane
        if (self._left_lane):
            if (self._left_lane.id == l_id):
                return self._left_lane
        return None
    
    
        
@dataclass        
class MP:
    #Maneuver Parameter that supports sampling
    value:float = 0.0               #target value
    bound_p:float = 0.0             #boundary in percentage (relative to value)
    nsamples:int = 1                #number of samples
    sigma:float = 1                 #std dev for sampling from normal
    
    def bounds(self):
        if (self.bound_p > 0.0):
            lo = self.value - (self.value * self.bound_p / 100)
            up = self.value + (self.value * self.bound_p / 100)
            return lo,up
        else: 
            return self.value,self.value

    def get_linear_samples(self):
        lo,up = self.bounds()
        if (lo == up):
            return (tuple(lo))
        return np.linspace(lo,up, self.nsamples)

    def get_uniform_samples(self):
        lo,up = self.bounds()
        samples = []
        for x in range(self.nsamples):
            samples.append(random.uniform(lo, up))
        return samples

    def get_normal_samples(self):
        samples = []
        for x in range(self.nsamples):
            samples.append(random.gauss(self.value, self.sigma))
        return samples

@dataclass        
class MVelKeepConfig:
    vel:MP = MP(14.0,10,6)          #velocity in [m/s] as MP
    time:MP = MP(4.0,10,3)          #duration in [s] as MP

@dataclass
class MStopConfig:
    #target
    time:MP = MP(5.0,10,6)          #[s]
    distance:MP = MP(10.0,10,6)     #[s]
    decel:MP = MP(9.0)
    #during
    min_decel:float = 9.0           #[g]
    max_decel:float = 9.0           #[g]

@dataclass
class MStopAtConfig:
    #target
    stop_pos:float                  #pos in s [m]
    distance:float = 10             #[m]

@dataclass
class MFollowConfig:
    #target
    target_vid:int                  #target vehicle id
    time:MP = MP(3.0,10,6)          #duration in [s] as MP
    time_gap:float = 2.0            #[s]
    #distance:float = 20             #distance from target [m]
    #constraints
    max_ttc:float = 10.0            #max time to collision [s]

@dataclass
class MLaneSwerveConfig:
    #target
    target_lid:int                  #target lane id
    time:MP = MP(5.0,10,6)          #target time in [s] as MP
    #soft constraints
    

@dataclass
class MCutInConfig:
    #target
    target_vid:int                  #target vehicle id
    vel:float = 30
    time:float = 5
    delta_s:tuple = (10,5,0)        #(s, vel, acc)
    delta_d:tuple = (0,5,0)         #(s, vel, acc)
    delta_cross:tuple = (10,5,0)    #(s, vel, acc)
    #soft constraints
    vel_dev:float = 10              #%
    time_dev:float = 10             #%



#TARGET_VELOCITY = 30.0 / 3.6 #=8.3m/s      #HV target speed [m/s] 
#TS_SAMP = 5.0 / 3.6  # ~1.38 m/s        #HV target speed sampling [m/s]
#VELOCITY_STEP = 6
#MAX_VELOCITY = 30 / 3.6                 #11 m/s
#MIN_VELOCITY = 28 / 3.6
#MAX_ACCEL = 2.0

#PLANNING TIMES
#PLAN_TIME_STEP = 0.5    #general planning time step
#VK_TIME = 4             #velocity keeping planning time 
#VK_MIN_TIME = 2         #velocity keeping time variation (+ and -)
#VK_MAX_TIME = 6

#FL_TIME = 4             #following planning time
#FL_MIN_TIME = 3         #following time variation (+ and -)
#FL_MAX_TIME = 5

#LC_TIME = 5
#STOP_DISTANCE_STEP = 10
#LC_TIME_VAR = 4
#CI_TIME = 8
#CI_TIME_VAR = 2 

#space sampling
#N_SAMPLES = 10
#SIGMA_S = [10.0, 2.0, 1.0] # s, s_d, s_dd
#SIGMA_D = [0.5, 0.5, 0.5]
#SIGMA_T = 2.0
