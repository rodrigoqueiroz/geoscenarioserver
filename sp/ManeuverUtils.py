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
from util.Transformations import normalize


def has_reached_point(pedestrian_state, point, threshold=1):
    """ Checks if the pedestrian has reached a given point in the cartesian frame
        @param point: Array [x,y] node position
        @param threshold: Max acceptable distance to point
    """
    pedestrian_pos = np.array([pedestrian_state.x, pedestrian_state.y])

    return np.linalg.norm(np.asarray(point) - pedestrian_pos) < threshold

def dir_to_follow_lane_border(pedestrian_state, current_lane, curr_waypoint):
    pedestrian_pos = np.array([pedestrian_state.x, pedestrian_state.y])

    # find entrance and exit points of the current lanelet
    entrance_pt_left = np.array([current_lane.leftBound[0].x, current_lane.leftBound[0].y])
    entrance_pt_right = np.array([current_lane.rightBound[0].x, current_lane.rightBound[0].y])
    exit_pt_left = np.array([current_lane.leftBound[-1].x, current_lane.leftBound[-1].y])
    exit_pt_right = np.array([current_lane.rightBound[-1].x, current_lane.rightBound[-1].y])

    entrance_pt = (entrance_pt_left + entrance_pt_right) / 2
    exit_pt = (exit_pt_left + exit_pt_right) / 2

    left_border = [np.array([pt.x, pt.y]) for pt in current_lane.leftBound]
    right_border = [np.array([pt.x, pt.y]) for pt in current_lane.rightBound]

    # determine which direction ped is heading in lanelet and flip order of border nodes if necessary
    if np.linalg.norm(entrance_pt - curr_waypoint) < np.linalg.norm(exit_pt - curr_waypoint):
        temp = entrance_pt
        entrance_pt = exit_pt
        exit_pt = temp

        left_border = [np.array([pt.x, pt.y]) for pt in list(reversed(current_lane.rightBound))]
        right_border = [np.array([pt.x, pt.y]) for pt in list(reversed(current_lane.leftBound))]

    pos_to_exit = exit_pt - pedestrian_pos
    pos_to_waypt = curr_waypoint - pedestrian_pos

    # determine which border (left or right) ped should follow from angle to their waypoint
    angle = np.arctan2(np.linalg.det(np.array([pos_to_exit, pos_to_waypt])), np.dot(pos_to_exit, pos_to_waypt))

    if angle > 0:
        # follow left border
        follow_border = left_border
    else:
        # follow right border
        follow_border = right_border

    # find first border segment the ped has not yet passed and return a parallel direction
    for idx in range(len(follow_border)-1):
        border_seg = follow_border[idx+1] - follow_border[idx]
        pos_to_border_start = pedestrian_pos - follow_border[idx]
        t = np.dot(border_seg, pos_to_border_start) / np.dot(border_seg, border_seg)

        if t < 1:
            return normalize(border_seg)

    # return direction parallel to last border segment if ped has passed all segments
    return normalize(follow_border[-1] - follow_border[-2])

def in_crosswalk_area(planner_state):
    cur_ll = planner_state.lanelet_map.get_occupying_lanelet_by_participant(planner_state.pedestrian_state.x, planner_state.pedestrian_state.y, "pedestrian")
    if cur_ll == None:
        return False
    return cur_ll.attributes["subtype"] == "crosswalk"


def point_in_rectangle(P, A, B, C, D):
    ''' Checks if the point pt is in the rectangle defined by
        points A, B, C, D in a counterclockwise orientation
    '''
    left_of_AB = (B[0]-A[0])*(P[1]-A[1]) - (B[1]-A[1])*(P[0]-A[0]) > 0
    left_of_BC = (C[0]-B[0])*(P[1]-B[1]) - (C[1]-B[1])*(P[0]-B[0]) > 0
    left_of_CD = (D[0]-C[0])*(P[1]-C[1]) - (D[1]-C[1])*(P[0]-C[0]) > 0
    left_of_DA = (A[0]-D[0])*(P[1]-D[1]) - (A[1]-D[1])*(P[0]-D[0]) > 0

    '''
    Could not directly return (left_of_AB and left_of_BC and left_of_CD and left_of_DA)
    for some reason so I had to split into two different bools (first_two and last_two).

    (left_of_AB and left_of_BC and left_of_CD and left_of_DA) apparently evaluates
    to an empty list instead of True or False
    '''
    first_two = left_of_AB and left_of_BC
    last_two = left_of_CD and left_of_DA
    inside_rect = first_two and last_two

    return inside_rect
