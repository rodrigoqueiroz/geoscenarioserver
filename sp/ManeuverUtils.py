#!/usr/bin/env python3
#rqueiroz@uwaterloo.ca
#d43sharm@uwaterloo.ca
#slarter@uwaterloo.ca
# ------------------------------------
# Util functions for decision-making
# ------------------------------------


from sp.ManeuverConfig import *
from SimConfig import *
import numpy as np
from Actor import *
import glog as log


def has_reached_goal(pedestrian_state, goal_point, threshold=1):
    """ Checks if the pedestrian has reached goal point in the cartesian frame
        @param goal_point: Array [x,y] goal position
        @param threshold: Max acceptable distance to goal
    """
    pedestrian_pos = np.array([pedestrian_state.x, pedestrian_state.y])

    return np.linalg.norm(np.asarray(goal_point) - pedestrian_pos) < threshold


def is_stopped(traffic_vehicle):
    return abs(traffic_vehicle.state.s_vel) < 0.05


