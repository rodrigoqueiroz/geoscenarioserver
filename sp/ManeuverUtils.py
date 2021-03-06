#!/usr/bin/env python3
#rqueiroz@uwaterloo.ca
#d43sharm@uwaterloo.ca
#slarter@uwaterloo.ca
# ------------------------------------
# Util functions for decision-making
# ------------------------------------


import numpy as np
from Actor import *
import glog as log
from SimTraffic import *
from sp.ManeuverConfig import *
from SimConfig import *


def has_reached_goal(pedestrian_state, goal_point, threshold=1):
    """ Checks if the pedestrian has reached goal point in the cartesian frame
        @param goal_point: Array [x,y] goal position
        @param threshold: Max acceptable distance to goal
    """
    pedestrian_pos = np.array([pedestrian_state.x, pedestrian_state.y])

    return np.linalg.norm(np.asarray(goal_point) - pedestrian_pos) < threshold


def is_stopped(traffic_vehicle):
    return abs(traffic_vehicle.state.s_vel) < 0.05


def in_crosswalk_area(pedestrian_state):
    cur_ll = self.sim_traffic.lanelet_map.get_occupying_lanelet_by_participant(pedestrian_state.x, pedestrian_state.y, "pedestrian")
    ret = cur_ll.attributes["subtype"] == "crosswalk"
    log.info(ret)
    return ret

def past_crosswalk_halfway(pedestrian_state):
    return False
