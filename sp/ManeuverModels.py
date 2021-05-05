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


def plan_maneuver(man_key, mconfig, sp, pedestrian_state, pedestrian_speed, lanelet_of_curr_waypoint, vehicles, pedestrians):
    #Micro maneuver layer
    if (man_key == Maneuver.M_KEEPINLANE):
        return plan_keep_in_lane(mconfig, sp, pedestrian_state, pedestrian_speed, lanelet_of_curr_waypoint, vehicles, pedestrians)
    elif (man_key == Maneuver.M_STOP):
        return plan_stop(mconfig, sp, pedestrian_state, pedestrian_speed, lanelet_of_curr_waypoint, vehicles, pedestrians)
    elif (man_key == Maneuver.M_UPDATEWAYPOINT):
        return plan_update_waypoint(mconfig, sp, pedestrian_state, pedestrian_speed, lanelet_of_curr_waypoint, vehicles, pedestrians)
    elif (man_key == Maneuver.M_ENTERCROSSWALK):
        return plan_enter_crosswalk(mconfig, sp, pedestrian_state, pedestrian_speed, lanelet_of_curr_waypoint, vehicles, pedestrians)
    elif (man_key == Maneuver.M_EXITCROSSWALK):
        return plan_exit_crosswalk(mconfig, sp, pedestrian_state, pedestrian_speed, lanelet_of_curr_waypoint, vehicles, pedestrians)


def plan_keep_in_lane(mconfig:MKeepInLaneConfig, sp, pedestrian_state:PedestrianState, pedestrian_speed, lanelet_of_curr_waypoint, vehicles=None, pedestrians=None):
    """
    KEEP IN LANE
    """
    pedestrian_pos = np.array([pedestrian_state.x, pedestrian_state.y])
    direction = normalize(sp.waypoint - pedestrian_pos)

    #
    # if sp.current_lanelet and sp.current_lanelet.attributes["type"] == "lanelet":
    #     if sp.current_lanelet != lanelet_of_curr_waypoint and np.linalg.norm(pedestrian_pos - sp.waypoint) > 2:
    #         direction = dir_to_follow_lane_border(pedestrian_state, sp.current_lanelet, sp.waypoint)

    sp.border_forces = False

    return direction, sp.waypoint, pedestrian_speed['default_desired']


def plan_stop(mconfig:MStopConfig, sp, pedestrian_state:PedestrianState, pedestrian_speed, lanelet_of_curr_waypoint, vehicles=None, pedestrians=None):
    """
    STOP MANEUVER
    """
    pedestrian_pos = np.array([pedestrian_state.x, pedestrian_state.y])
    direction = normalize(sp.waypoint - pedestrian_pos)

    sp.border_forces = False

    return direction, sp.waypoint, 0.0


def plan_update_waypoint(mconfig:MUpdateWaypointConfig, sp, pedestrian_state:PedestrianState, pedestrian_speed, lanelet_of_curr_waypoint, vehicles=None, pedestrians=None):
    """
    UPDATE INTERMEDIATE WAYPOINT
    """
    pedestrian_pos = np.array([pedestrian_state.x, pedestrian_state.y])
    direction = normalize(sp.waypoint - pedestrian_pos)

    sp.border_forces = False

    return direction, next(sp.route), pedestrian_speed['default_desired']

def plan_enter_crosswalk(mconfig:MEnterCrosswalkConfig, sp, pedestrian_state:PedestrianState, pedestrian_speed, lanelet_of_curr_waypoint, vehicles=None, pedestrians=None):
    """
    ENTER CROSSWALK
    """
    pedestrian_pos = np.array([pedestrian_state.x, pedestrian_state.y])
    waypoint = sp.target_crosswalk_pts[1]

    direction = normalize(waypoint - pedestrian_pos)

    sp.border_forces = False

    return direction, waypoint, pedestrian_speed['default_desired']

def plan_exit_crosswalk(mconfig:MExitCrosswalkConfig, sp, pedestrian_state:PedestrianState, pedestrian_speed, lanelet_of_curr_waypoint, vehicles=None, pedestrians=None):
    """
    EXIT CROSSWALK
    """
    pedestrian_pos = np.array([pedestrian_state.x, pedestrian_state.y])
    sp.sp_planner.plan_local_route()
    direction = normalize(sp.waypoint - pedestrian_pos)

    sp.border_forces = False

    return direction, sp.waypoint, pedestrian_speed['default_desired']
