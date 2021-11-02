#!/usr/bin/env python3
#rqueiroz@uwaterloo.ca
# ---------------------------------------------
# CONFIG Data Classes and Constants for Maneuvers
# --------------------------------------------
from __future__ import annotations  #Must be first Include. Will be standard in Python4
from dataclasses import dataclass
from SimConfig import *
from util.Utils import *
from enum import Enum, IntEnum

import numpy as np
from typing import Dict


class Maneuver(Enum):
    M_VELKEEP = 1
    M_FOLLOW = 2
    M_LANESWERVE = 3
    M_CUTIN = 4
    M_STOP = 5
    M_REVERSE = 6

class SamplingMethod(IntEnum):
    LINEAR = 1      #linear space
    UNIFORM = 2     #random from uniform distribution
    NORMAL = 3      #random from gaussian


@dataclass
class LaneConfig:
    ''' Lane Config in Frenet Frame
        Left and Right lanes exist only if traversal is possible
    '''
    id:int = 0
    max_velocity:float = 30         # [m/s]
    left_bound:float = 2            #d
    right_bound:float = -2          #d
    _left_lane: LaneConfig = None
    _right_lane: LaneConfig = None
    stopline_pos:float = None

    def get_current_lane(self, d):
        if self.right_bound <= d <= self.left_bound:
            return self
        elif self._right_lane and self._right_lane.right_bound <= d < self._right_lane.left_bound:
            return self._right_lane
        elif self._left_lane and self._left_lane.right_bound <= d < self._left_lane.left_bound:
            return self._left_lane
        return None

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
    
    def get_central_d(self):
        return (self.left_bound - self.right_bound)/2 + self.right_bound

@dataclass
class LT:
    '''
    Lateral target configuration for sampling in the lane width space.
    @param target: distance [in meters] from the lane centre ( < 0 is right, > 0 is left)
    The target value will be used exclusevely if a single sample is used (nsamples=1) 
    or as the mean if the sampling method is NORMAL (sigma as standard deviation).
    If nsamples > 1 and sampling is LINEAR or NORMAL, all the available lateral space is used.
    Note that limit_lane_width is important to keep the lateral space within lane boundaries.
    This constraint can be removed when a single target value is used or samplig with NORMAL.
    '''
    target:float = 0.0                      #lateral target position in meters (0.0 = center of the lane)
    nsamples:int = 1                        #number of sampling points on the lane width for lateral planning
    sampling:int = SamplingMethod.NORMAL    #sampling method
    sigma:float = 1.0                       #std dev for sampling from normal
    limit_lane_width:bool = True            #Limit sampling values by lane width. If False, should be only used with NORMAL sampling
    limit_vehicle_size:bool = True          #Limit sampling values by vehicle size (within boundaries)

    def get_samples(self, lane_config:LaneConfig):
        target = lane_config.get_central_d() + self.target
        if (self.nsamples<=1):
            return tuple([target])
        
        if self.limit_lane_width:     
            up = lane_config.left_bound
            lo = lane_config.right_bound
        else:
            up = lane_config._left_lane.left_bound if lane_config._left_lane else lane_config.left_bound
            lo = lane_config._right_lane.right_bound if lane_config._right_lane else lane_config.right_bound
        
        vsize = VEHICLE_RADIUS if self.limit_vehicle_size else 0
        up = up - vsize
        lo = lo + vsize
        
        if (self.sampling== SamplingMethod.LINEAR):
            return linear_samples(self.nsamples,lo,up)
        elif (self.sampling== SamplingMethod.UNIFORM):
            return uniform_samples(self.nsamples,lo,up)
        elif (self.sampling== SamplingMethod.NORMAL):
            return normal_samples(self.nsamples,target,self.sigma,lo,up)
        return tuple([0])


@dataclass
class MP:
    '''
    Maneuver Parameter that supports sampling (positve only)
    '''
    value:float = 0.0               #target value (min is 0.0)
    bound_p:float = 0.0             #boundary in percentage (+/- relative to value)
    nsamples:int = 1                #number of samples
    sampling:int = SamplingMethod.UNIFORM
    sigma:float = 1                 #std dev for sampling from normal
    
    def get_samples(self):
        if self.nsamples<=1 or self.bound_p==0.0:
            return tuple([self.value])
        lo = self.value - (self.value * self.bound_p / 100)
        up = self.value + (self.value * self.bound_p / 100)
        if lo < 0: 
            lo = 0.0
        
        if (self.sampling==SamplingMethod.LINEAR):
            return linear_samples(self.nsamples,lo,up)
        elif (self.sampling==SamplingMethod.UNIFORM):
            return uniform_samples(self.nsamples,lo,up)
        elif (self.sampling==SamplingMethod.NORMAL):
            return normal_samples(self.nsamples,self.value,self.sigma,lo,up)

@dataclass
class MConfig:
    ''''
    Maneuver Configuration SuperClass.
    Individual cost must be changed in the __post_init__ method
    '''
    #feasibility_weight defines what functions will be used
    #to remove trajectories from candidate set [binary 0-1]
    feasibility_constraints: Dict = field(default_factory=lambda:{
        'max_lat_jerk':        1,
        'max_long_jerk':       1,
        'max_long_acc':        1,
        'max_lat_acc':         1,
        'collision':           1,
        'off_lane':            1,
        'direction':           1,
    })
    #cost_weight defines what cost functions will be used
    #to select trajectories from candidate set [cost > 0]
    cost_weight: Dict = field(default_factory=lambda:{
        'time_cost':                1,
        'effic_cost':               1,
        'lane_offset_cost':         1,
        'total_long_jerk_cost':     1,
        'total_lat_jerk_cost':      1,
        'total_long_acc_cost':      1,
        'total_lat_acc_cost':       1,
        'proximity_cost':           10, #10 
        
    })

    #Cost thresholds
    expected_offset_per_sec:float = 0.5   # [m]
    expected_long_acc_per_sec:float = 1   # [m/s/s]
    expected_lat_acc_per_sec:float = 1    # [m/s/s]
    expected_long_jerk_per_sec:float = 2  # [m/s/s/s]
    expected_lat_jerk_per_sec:float = 2   # [m/s/s/s]

    #Feasibility constraints
    max_long_jerk:float = 10.0              # maximum longitudinal jerk [m/s/s/s]
    max_lat_jerk:float = 10.0               # maximum lateral jerk [m/s/s/s]
    max_long_acc:float = 12.0              # maximum longitudinal acceleration [m/s/s]
    max_lat_acc:float = 4.9               # maximum lateral acceleration [m/s/s]
    
    #Lateral lane target. By default, targets center
    lat_target:LT = LT(0.0,1)

    #Precision defines how feasibility and costs are computed (and how integrals are approximated). 
    #Higher(100) = better precision, but impacts performance. 
    #Use with caution when multiple vehicles are used in simulation
    cost_precision:float = 10             #from 10 to 100.

@dataclass
class MVelKeepConfig(MConfig):
    vel:MP = MP(14.0,10,3)              #velocity in [m/s] as MP
    time:MP = MP(3.0,20,6)              #duration in [s] as MP
    max_diff:float = 8.0                 #max vel diff (current to target).
    mkey:int = Maneuver.M_VELKEEP

@dataclass
class MReverseConfig(MConfig):
    vel:MP = MP(5.0,10,6)           #velocity in [m/s] as MP
    time:MP = MP(3.0,20,3)          #duration in [s] as MP
    mkey:int = Maneuver.M_REVERSE

    def __post_init__(self):
        self.feasibility_constraints['direction'] = 0

@dataclass
class MStopConfig(MConfig):
    class StopTarget(IntEnum):
        NOW = 0         #stop with no particular position. Use decel to adjust behavior.
        S_POS = 1       #stop at a particular position in the frenet frame
        GOAL = 2        #stop at the vehicle's goal point
        STOP_LINE = 3   #stop at the stop line of a regulatory element, if any applies

    #target
    target:int = StopTarget.GOAL #Aim at a given position (GOAL, STOP_LINE, STOP_POS) or stop NOW.
    pos:float = 0.0                 #pos in s [m]
    distance:float = 0.0            #distance to target pos in [m]
    #time:MP = MP(3.0,40,6)         #[s]
    mkey:int = Maneuver.M_STOP

    def __post_init__(self):
        self.max_long_acc = 30.0
    

@dataclass
class MFollowConfig(MConfig):
    #target
    target_vid:int = None           #target vehicle id
    time:MP = MP(4.0,50,10)         #duration in [s] as MP
    time_gap:float = 3.0            #[s]
    stop_distance:float = 3.0       #target distance when lead vehicle stops
    mkey:int = Maneuver.M_FOLLOW

@dataclass
class MLaneSwerveConfig(MConfig):
    #target
    target_lid:int = None           #target lane id
    time:MP = MP(4.2,10,6)          #target time in [s] as MP
    mkey:int = Maneuver.M_LANESWERVE

    def __post_init__(self):
        self.cost_weight['lane_offset_cost'] = 0.5
        self.feasibility_constraints['off_lane'] = 0

@dataclass
class MCutInConfig(MConfig):
    #target
    target_vid:int = None               #target vehicle id
    target_lid:int = None
    time:MP = MP(4.0,10,6)
    delta_s:tuple = (10,5,0)        #(s, vel, acc)
    delta_s_sampling = [(10,5), (0,1), (0,1)]
    delta_d:tuple = (0,0,0)         #(d, vel, acc)
    mkey:int = Maneuver.M_CUTIN

    def __post_init__(self):
        self.cost_weight['lane_offset_cost'] = 0.5
        self.feasibility_constraints['off_lane'] = 0
