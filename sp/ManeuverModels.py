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


def plan_maneuver(man_key, mconfig, sp, pedestrian_state, pedestrian_speed, target_crosswalk, vehicles, pedestrians, previous_maneuver):
    #Micro maneuver layer
    if (man_key == Maneuver.M_KEEPINLANE):
        return plan_keep_in_lane(mconfig, sp, pedestrian_state, pedestrian_speed, target_crosswalk)
    elif (man_key == Maneuver.M_STOP):
        return plan_stop(mconfig, sp, pedestrian_state, pedestrian_speed, target_crosswalk)
    elif (man_key == Maneuver.M_ENTERCROSSWALK):
        return plan_enter_crosswalk(mconfig, sp, pedestrian_state, pedestrian_speed, target_crosswalk, previous_maneuver)
    elif (man_key == Maneuver.M_EXITCROSSWALK):
        return plan_exit_crosswalk(mconfig, sp, pedestrian_state, pedestrian_speed, target_crosswalk, previous_maneuver)
    elif (man_key == Maneuver.M_WAITATCROSSWALK):
        return plan_wait_at_crosswalk(mconfig, sp, pedestrian_state, pedestrian_speed, target_crosswalk)


def plan_keep_in_lane(mconfig:MKeepInLaneConfig, sp, pedestrian_state:PedestrianState, pedestrian_speed, target_crosswalk):
    """
    KEEP IN LANE
    """
    pedestrian_pos = np.array([pedestrian_state.x, pedestrian_state.y])
    direction = normalize(sp.current_waypoint - pedestrian_pos)

    if sp.current_lanelet and sp.current_lanelet.attributes["type"] == "lanelet":
        if np.linalg.norm(pedestrian_pos - sp.current_waypoint) > 2:
            direction = dir_to_follow_lane_border(pedestrian_state, sp.current_lanelet, sp.current_waypoint, sp.sp_planner.inverted_path)

    return direction, sp.current_waypoint, pedestrian_speed['default_desired']


def plan_stop(mconfig:MStopConfig, sp, pedestrian_state:PedestrianState, pedestrian_speed, target_crosswalk):
    """
    STOP MANEUVER
    """
    pedestrian_pos = np.array([pedestrian_state.x, pedestrian_state.y])
    direction = normalize(sp.current_waypoint - pedestrian_pos)

    return direction, sp.current_waypoint, 0.0


def plan_enter_crosswalk(mconfig:MEnterCrosswalkConfig, sp, pedestrian_state:PedestrianState, pedestrian_speed, target_crosswalk, previous_maneuver):
    """
    ENTER CROSSWALK
    """
    pedestrian_pos = np.array([pedestrian_state.x, pedestrian_state.y])
    waypoint = target_crosswalk['exit']

    direction = normalize(waypoint - pedestrian_pos)

    return direction, waypoint, pedestrian_speed['default_desired']

def plan_exit_crosswalk(mconfig:MExitCrosswalkConfig, sp, pedestrian_state:PedestrianState, pedestrian_speed, target_crosswalk, previous_maneuver):
    """
    EXIT CROSSWALK
    """
    if previous_maneuver != Maneuver.M_EXITCROSSWALK:
        # get point just outside of crosswalk to plan next path from
        if sp.sp_planner.inverted_path:
            xwalk = sp.path[0].invert()
        else:
            xwalk = sp.path[-1]

        second_last_left_pt = np.array([xwalk.leftBound[-2].x, xwalk.leftBound[-2].y])
        second_last_right_pt = np.array([xwalk.rightBound[-2].x, xwalk.rightBound[-2].y])
        last_left_pt = np.array([xwalk.leftBound[-1].x, xwalk.leftBound[-1].y])
        last_right_pt = np.array([xwalk.rightBound[-1].x, xwalk.rightBound[-1].y])

        left_pt = last_left_pt + normalize(last_left_pt - second_last_left_pt)*0.25
        right_pt = last_right_pt + normalize(last_right_pt - second_last_right_pt)*0.25

        mid = (left_pt + right_pt) / 2

        sp.sp_planner.plan_local_path(mid)

    pedestrian_pos = np.array([pedestrian_state.x, pedestrian_state.y])
    direction = normalize(sp.current_waypoint - pedestrian_pos)

    return direction, sp.current_waypoint, pedestrian_speed['default_desired']

def plan_wait_at_crosswalk(mconfig:MStopConfig, sp, pedestrian_state:PedestrianState, pedestrian_speed, target_crosswalk):
    """
    WAIT AT CROSSWALK MANEUVER
    """
    pedestrian_pos = np.array([pedestrian_state.x, pedestrian_state.y])
    direction = normalize(sp.current_waypoint - pedestrian_pos)

    return direction, sp.current_waypoint, 0.0
