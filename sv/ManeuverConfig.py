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
import random
import numpy as np


class Maneuver(Enum):
    M_VELKEEP = 1
    M_FOLLOW = 2
    M_LANESWERVE = 3
    M_CUTIN = 4
    M_STOP = 5
    M_STOP_AT = 6
    M_REVERSE = 7

class StopAtType(IntEnum):
    S_POS = 1        #stop at a particular position in the frenet frame
    GOAL = 2         #stop at the vehicle's goal point
    STOP_LINE = 3    #stop at the stop line of a regulatory element, if any applies

class SamplingMethod(Enum):
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
    left_bound:float = 4            #d
    right_bound:float = 0           #d
    _left_lane: LaneConfig = None
    _right_lane: LaneConfig = None
    nsamples:int = 2                #number of sampling points on the lane width for lateral planning
    sampling:int = SamplingMethod.UNIFORM
    sigma:float = 1                 #std dev for sampling from normal

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

    def get_samples(self):
        if (self.nsamples==1):
            return tuple([0])

        if (self.sampling==SamplingMethod.LINEAR):
            return self.get_linear_samples()
        elif (self.sampling==SamplingMethod.UNIFORM):
            return self.get_uniform_samples()
        elif (self.sampling==SamplingMethod.NORMAL):
            return self.get_normal_samples()

        return tuple([0])

    def get_linear_samples(self):
        lo = self.right_bound + VEHICLE_RADIUS
        up = self.left_bound - VEHICLE_RADIUS
        if (lo >= up):
            return tuple([lo])
        return np.linspace(lo,up, self.nsamples)

    def get_uniform_samples(self):
        lo = self.right_bound + VEHICLE_RADIUS
        up = self.left_bound - VEHICLE_RADIUS
        samples = []
        for x in range(self.nsamples):
            samples.append(random.uniform(lo, up))
        return samples

    def get_normal_samples(self):
        lo = self.right_bound + VEHICLE_RADIUS
        up = self.left_bound - VEHICLE_RADIUS
        sd = self.sigma
        samples = []
        for x in range(self.nsamples):
            #same as random.gauss(), but with bounds:
            s = get_truncated_normal(0,sd,lo,up)
            samples.append(s)
        return samples


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
    feasibility_constraints = {
        'max_lat_jerk':        0,
        'max_long_jerk':       0,
        'max_acc':             0,
        'collision':           0,
        'off_lane':            1,
    }
    #cost_weight defines what cost functions will be used
    #to select trajectories from candidate set [cost > 0]
    cost_weight = {
        'time_cost':                1,
        'effic_cost':               1,
        'lane_offset_cost':         1,
        'total_long_jerk_cost':     1,
        'total_lat_jerk_cost':      1,
        'total_long_acc_cost':      1,
        'total_lat_acc_cost':       1,
        'proximity_cost':           0,
    }
    expected_offset_per_sec = 0.5   # [m]
    expected_long_acc_per_sec = 1   # [m/s/s]
    expected_lat_acc_per_sec = 1    # [m/s/s]
    expected_long_jerk_per_sec = 2  # [m/s/s/s]
    expected_lat_jerk_per_sec = 2   # [m/s/s/s]

    #Feasibility constraints
    max_long_jerk = 10              # maximum longitudinal jerk [m/s/s/s]
    max_lat_jerk = 10               # maximum lateral jerk [m/s/s/s]
    max_long_acc = 5.0              # maximum longitudinal acceleration [m/s/s]
    max_lat_acc = 4.9               # maximum lateral acceleration [m/s/s]


@dataclass
class MVelKeepConfig(MConfig):
    vel:MP = MP(10.0,10,6)              #velocity in [m/s] as MP
    time:MP = MP(3.0,10,3)              #duration in [s] as MP
    time_lowvel:MP = MP(10.0,10,3)       #duration in [s] as MP when starting
    vel_threshold:float = 6             #upper bound for lowvel in [m/s]
    mkey:int = Maneuver.M_VELKEEP

@dataclass
class MReverseConfig(MConfig):
    vel:MP = MP(5.0,10,6)           #velocity in [m/s] as MP
    time:MP = MP(4.0,10,3)          #duration in [s] as MP
    mkey:int = Maneuver.M_REVERSE

@dataclass
class MStopConfig(MConfig):
    #target
    time:MP = MP(3.0,10,6)          #[s]
    distance:MP = MP(10.0,10,6)     #[s]
    decel:MP = MP(9.0)
    #during
    min_decel:float = 9.0           #[g]
    max_decel:float = 9.0           #[g]
    mkey:int = Maneuver.M_STOP

@dataclass
class MStopAtConfig(MConfig):
    #target
    time:MP = MP(3.0,10,6)          #[s]
    stop_pos:float = 0              #pos in s [m]
    mkey:int = Maneuver.M_STOP_AT
    stop_type:int = StopAtType.GOAL

@dataclass
class MFollowConfig(MConfig):
    #target
    target_vid:int = None           #target vehicle id
    time:MP = MP(3.0,10,6)          #duration in [s] as MP
    time_gap:float = 2.0            #[s]
    decel:MP = MP(5.0,10,6)         #[m/s2]
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
    delta_d:tuple = (0,0,0)         #(d, vel, acc)
    mkey:int = Maneuver.M_CUTIN

    def __post_init__(self):
        self.cost_weight['lane_offset_cost'] = 0.5
        self.feasibility_constraints['off_lane'] = 0
