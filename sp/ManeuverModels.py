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


def plan_maneuver(man_key, mconfig, pedestrian_state, pedestrian_speed, route, curr_route_node, vehicles, pedestrians):
    #Micro maneuver layer
    if (man_key == Maneuver.M_VELKEEP):
        return plan_velocity_keeping(pedestrian_state, pedestrian_speed, route, curr_route_node, mconfig, vehicles, pedestrians)
    elif (man_key == Maneuver.M_STOP):
        return plan_stop(pedestrian_state, pedestrian_speed, route, curr_route_node, mconfig, vehicles, pedestrians)
    elif (man_key == Maneuver.M_UPDATEWAYPOINT):
        return plan_update_waypoint(pedestrian_state, pedestrian_speed, route, curr_route_node, mconfig, vehicles, pedestrians)


def plan_velocity_keeping(pedestrian_state:PedestrianState, pedestrian_speed, route, curr_route_node, mconfig:MVelKeepConfig, vehicles=None,  pedestrians=None):
    """
    VELOCITY KEEPING
    No target point, but needs to adapt to a desired velocity
    """

    return curr_route_node, pedestrian_speed['default_desired']


def plan_stop(pedestrian_state:PedestrianState, pedestrian_speed, route, curr_route_node, mconfig:MStopConfig, vehicles=None, pedestrians=None):
    """
    STOP MANEUVER
    """
    return curr_route_node, 0.0


def plan_update_waypoint(pedestrian_state:PedestrianState, pedestrian_speed, route, curr_route_node, mconfig:MUpdateWaypoint, vehicles=None, pedestrians=None):
    """
    UPDATE INTERMEDIATE WAYPOINT
    """
    return curr_route_node + 1, None
