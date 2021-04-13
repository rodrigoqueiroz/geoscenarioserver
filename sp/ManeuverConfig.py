#!/usr/bin/env python3
#rqueiroz@uwaterloo.ca
#slarter@uwaterloo.ca
# ---------------------------------------------
# CONFIG Data Classes and Constants for Maneuvers
# --------------------------------------------
from __future__ import annotations  #Must be first Include. Will be standard in Python4
from dataclasses import dataclass
from SimConfig import *
from util.Utils import *
from enum import Enum, IntEnum
import random
import numpy as np
from typing import Dict


class Maneuver(Enum):
    M_VELKEEP = 1
    M_STOP = 2
    M_UPDATEWAYPOINT = 3


class SamplingMethod(Enum):
    LINEAR = 1      #linear space
    UNIFORM = 2     #random from uniform distribution
    NORMAL = 3      #random from gaussian


@dataclass
class MP:
    '''
    Maneuver Parameter that supports sampling
    '''
    value:float = 0.0               #target value
    bound_p:float = 0.0             #boundary in percentage (+/- relative to value)
    nsamples:int = 1                #number of samples
    sampling:int = SamplingMethod.UNIFORM
    sigma:float = 1                 #std dev for sampling from normal

    def get_samples(self):
        if (self.nsamples==1):
            return tuple([value])
        elif (self.bound_p==0.0):
            return tuple([value])
        elif (self.sampling==SamplingMethod.LINEAR):
            return self.get_linear_samples()
        elif (self.sampling==SamplingMethod.UNIFORM):
            return self.get_uniform_samples()
        elif (self.sampling==SamplingMethod.NORMAL):
            return self.get_normal_samples()

        return tuple([self.value])

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
            return tuple([lo])
        return np.linspace(lo,up, self.nsamples)

    def get_uniform_samples(self):
        lo,up = self.bounds()
        samples = []
        for x in range(self.nsamples):
            samples.append(random.uniform(lo, up))
        return samples

    def get_normal_samples(self):
        lo,up = self.bounds()
        sd = self.sigma
        samples = []
        for x in range(self.nsamples):
            #same as random.gauss(), but with bounds:
            s = get_truncated_normal(0,sd,lo,up)
            samples.append(s)
        return samples


@dataclass
class MConfig:
    ''''
    Maneuver Configuration SuperClass.
    Individual cost must be changed in the __post_init__ method
    '''
    #feasibility_weight defines what functions will be used
    #to remove trajectories from candidate set [binary 0-1]
    feasibility_constraints: Dict = field(default_factory=lambda:{
        'max_lat_jerk':        0,
        'max_long_jerk':       0,
        'max_long_acc':        1,
        'max_lat_acc':         0,
        'collision':           1,
        'off_lane':            1,
        'direction':           0,
    })
    #cost_weight defines what cost functions will be used
    #to select trajectories from candidate set [cost > 0]
    cost_weight: Dict = field(default_factory=lambda:{
        'time_cost':                1,
        'effic_cost':               1,
        'lane_offset_cost':         3,
        'total_long_jerk_cost':     1,
        'total_lat_jerk_cost':      1,
        'total_long_acc_cost':      1,
        'total_lat_acc_cost':       1,
        'proximity_cost':           10, #10
        'progress_cost':            0,
    })
    expected_offset_per_sec = 0.5   # [m]
    expected_long_acc_per_sec = 1   # [m/s/s]
    expected_lat_acc_per_sec = 1    # [m/s/s]
    expected_long_jerk_per_sec = 2  # [m/s/s/s]
    expected_lat_jerk_per_sec = 2   # [m/s/s/s]

    #Feasibility constraints
    max_long_jerk = 10              # maximum longitudinal jerk [m/s/s/s]
    max_lat_jerk = 10               # maximum lateral jerk [m/s/s/s]
    max_long_acc = 6.0              # maximum longitudinal acceleration [m/s/s]
    max_lat_acc = 4.9               # maximum lateral acceleration [m/s/s]


@dataclass
class MVelKeepConfig(MConfig):
    vel:MP = MP(1.0,10,3)              #velocity in [m/s] as MP
    time:MP = MP(3.0,20,3)              #duration in [s] as MP
    #time_lowvel:MP = MP(6.0,10,3)      #duration in [s] as MP when starting
    #vel_threshold:float = 7            #upper bound for lowvel in [m/s]
    max_diff:float = 1.5               #max vel diff (current to target).
    mkey:int = Maneuver.M_VELKEEP


@dataclass
class MStopConfig(MConfig):
    class Type(IntEnum):
        NOW = 0         #stop with no particular position. Use decel to adjust behavior.
        S_POS = 1       #stop at a particular position
        GOAL = 2        #stop at the vehicle's goal point
        STOP_LINE = 3   #stop at the stop line of a regulatory element, if any applies

    #target
    type:int = Type.NOW            #Aim at a given position (GOAL, STOP_LINE, STOP_POS) or stop NOW.
    pos:float = 0.0                 #pos in s [m]
    distance:float = 0.0            #distance to stop_pos in [m]
    #time:MP = MP(3.0,40,6)         #[s]
    mkey:int = Maneuver.M_STOP

    def __post_init__(self):
        self.max_long_acc = 12.0


@dataclass
class MUpdateWaypoint(MConfig):
    class Type(IntEnum):
        NEXT = 0            # continue to next node in route
        XWALK_ENTRY = 1     # add entry of xwalk as new and next node in route
        XWALK_EXIT = 2      # add exit of xwalk as new and next node in route

    vel:MP = MP(1.0,10,3)              #velocity in [m/s] as MP
    time:MP = MP(3.0,20,3)              #duration in [s] as MP
    #time_lowvel:MP = MP(6.0,10,3)      #duration in [s] as MP when starting
    #vel_threshold:float = 7            #upper bound for lowvel in [m/s]
    max_diff:float = 1.5               #max vel diff (current to target).
    mkey:int = Maneuver.M_UPDATEWAYPOINT
