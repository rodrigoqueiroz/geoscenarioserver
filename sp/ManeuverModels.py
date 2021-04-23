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
from sp.ManeuverConfig import *
from SimConfig import *
from Actor import *
from typing import Callable
from sp.ManeuverUtils import *
from util.Transformations import normalize


def plan_maneuver(man_key, mconfig, pedestrian_state, pedestrian_speed, curr_waypoint, route, current_lanelet, vehicles, pedestrians):
    #Micro maneuver layer
    if (man_key == Maneuver.M_KEEPINLANE):
        return plan_keep_in_lane(pedestrian_state, pedestrian_speed, curr_waypoint, route, current_lanelet, mconfig, vehicles, pedestrians)
    elif (man_key == Maneuver.M_STOP):
        return plan_stop(pedestrian_state, pedestrian_speed, curr_waypoint, route, current_lanelet, mconfig, vehicles, pedestrians)
    elif (man_key == Maneuver.M_UPDATEWAYPOINT):
        return plan_update_waypoint(pedestrian_state, pedestrian_speed, curr_waypoint, route, current_lanelet, mconfig, vehicles, pedestrians)


def plan_keep_in_lane(pedestrian_state:PedestrianState, pedestrian_speed, curr_waypoint, route, current_lanelet, mconfig:MKeepInLaneConfig, vehicles=None,  pedestrians=None):
    """
    KEEP IN LANE
    No target point, but needs to adapt to a desired velocity
    """
    direction = dir_to_follow_lane_border(pedestrian_state, current_lanelet, curr_waypoint)

    return direction, curr_waypoint, pedestrian_speed['default_desired']


def plan_stop(pedestrian_state:PedestrianState, pedestrian_speed, curr_waypoint, route, current_lanelet, mconfig:MStopConfig, vehicles=None, pedestrians=None):
    """
    STOP MANEUVER
    """
    pedestrian_pos = np.array([pedestrian_state.x, pedestrian_state.y])
    direction = normalize(curr_waypoint-pedestrian_pos)

    return direction, curr_waypoint, 0.0


def plan_update_waypoint(pedestrian_state:PedestrianState, pedestrian_speed, curr_waypoint, route, current_lanelet, mconfig:MUpdateWaypoint, vehicles=None, pedestrians=None):
    """
    UPDATE INTERMEDIATE WAYPOINT
    """
    pedestrian_pos = np.array([pedestrian_state.x, pedestrian_state.y])
    direction = normalize(curr_waypoint-pedestrian_pos)
    
    return direction, next(route), pedestrian_speed['default_desired']
