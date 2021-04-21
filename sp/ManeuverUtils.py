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


def has_reached_point(pedestrian_state, point, threshold=1):
    """ Checks if the pedestrian has reached a given point in the cartesian frame
        @param point: Array [x,y] node position
        @param threshold: Max acceptable distance to point
    """
    pedestrian_pos = np.array([pedestrian_state.x, pedestrian_state.y])

    return np.linalg.norm(np.asarray(point) - pedestrian_pos) < threshold

def direction_to_center_of_lane(pedestrian_state, current_lane, next_waypoint):
        pedestrian_pos = np.array([pedestrian_state.x, pedestrian_state.y])

        '''
        1) determine which direction ped is walking in (which lane endpoint are they walking towards)
        2) starting from the other end, determine what is the last way on each border the ped can "drop a 90" to
            a) if they can't drop a 90 to any way, take either first or last node depending on which end point they are closer to
        3) direction points to the midpoint between the two end points of the selected ways on each border
        '''

        # 1)
        lane_entrance = (np.array(current_lane.leftBound[0][0]) + np.array(current_lane.rightBound[0][0])) / 2
        lane_exit = (np.array(current_lane.leftBound[-1][1]) + np.array(current_lane.rightBound[-1][1])) / 2
        if np.linalg.norm(next_waypoint - lane_exit) < np.linalg.norm(next_waypoint - lane_entrance):
            left_bound = current_lane.leftBound
            right_bound = current_lane.rightBound
        else:
            lane_entrance = (np.array(current_lane.leftBound[-1][1]) + np.array(current_lane.rightBound[-1][1])) / 2
            lane_exit = (np.array(current_lane.leftBound[0][0]) + np.array(current_lane.rightBound[0][0])) / 2
            left_bound = reverse([way.invert() for way in current_lane.rightBound])
            right_bound = reverse([way.invert() for way in current_lane.leftBound])

        # 2)
        closest_left_way = None
        closest_right_way = None

        for way in left_bound:
            way_vec = np.array(way[1]) - np.array(way[0])
            pos_to_way_start = pedestrian_pos - np.array(way[0])

            t = np.dot(way_vec, pos_to_way_start) / np.dot(way_vec, way_vec)

            # ped can "drop a 90" to way if 0<=t<=1
            if t >= 0 and t <= 1:
                closest_left_way = way

        for way in right_bound:
            way_vec = np.array(way[1]) - np.array(way[0])
            pos_to_way_start = pedestrian_pos - np.array(way[0])

            t = np.dot(way_vec, pos_to_way_start) / np.dot(way_vec, way_vec)

            if t >= 0 and t <= 1:
                closest_right_way = way

        if closest_left_way == None:
            if np.linalg.norm(pedestrian_pos - lane_entrance) < np.linalg.norm(pedestrian_pos - lane_exit):
                closest_left_way = left_bound[0]
            else:
                closest_left_way = left_bound[-1]

        if closest_right_way == None:
            if np.linalg.norm(pedestrian_pos - lane_entrance) < np.linalg.norm(pedestrian_pos - lane_exit):
                closest_right_way = right_bound[0]
            else:
                closest_right_way = right_bound[-1]

        # 3)
        target_pt = (np.array(closest_left_way) + np.array(closest_right_way)) / 2
        direction = target_pt - pedestrian_pos

        return direction

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
