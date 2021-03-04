#!/usr/bin/env python
#rqueiroz@uwaterloo.ca
#slarter@uwaterloo.ca
# --------------------------------------------
# GEOSCENARIO Micro Maneuver Models for Motion Planning
# --------------------------------------------
import numpy as np
from copy import copy
import itertools
import glog as log
#from multiprocessing import Pool as ThreadPool
from sp.ManeuverConfig import *
from SimConfig import *
from Actor import *
from typing import Callable
from sp.ManeuverUtils import *


def plan_maneuver(man_key, mconfig, pedestrian_state, vehicles, pedestrians):
    #Micro maneuver layer
    if (man_key == Maneuver.M_VELKEEP):
        return plan_velocity_keeping(pedestrian_state, mconfig, vehicles, pedestrians)
    elif (man_key == Maneuver.M_STOP):
        return plan_stop(pedestrian_state, mconfig, vehicles, pedestrians)


def plan_velocity_keeping(pedestrian_state:PedestrianState, mconfig:MVelKeepConfig, vehicles=None,  pedestrians = None):
    """
    VELOCITY KEEPING
    No target point, but needs to adapt to a desired velocity
    """
    ped_speed = np.linalg.norm([pedestrian_state.x_vel, pedestrian_state.y_vel])

    #cap target vel to maximum difference
    #this will smooth the trajectory when starting
    if (mconfig.vel.value - ped_speed) > mconfig.max_diff:
        target_speed = copy(mconfig.vel)
        target_speed.value = ped_speed + mconfig.max_diff
    else:
        target_speed =  mconfig.vel.value

    return target_speed


def plan_stop(pedestrian_state:PedestrianState, mconfig:MStopConfig, vehicles=None, pedestrians = None):
    return 0.0
