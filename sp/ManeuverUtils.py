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

def center_pt_of_current_lane(pedestrian_state, current_lane, curr_waypoint):
        pedestrian_pos = np.array([pedestrian_state.x, pedestrian_state.y])

        '''
        1) determine which direction ped is walking in (which lane endpoint are they walking towards)
        2) starting from the other end, determine what is the last way on each border the ped can "drop a 90" to
            a) if they can't drop a 90 to any way, take either first or last node depending on which end point they are closer to
        3) direction points to the midpoint between the two end points of the selected ways on each border
        '''

        # 1)
        entrance_pt_left = np.array([current_lane.leftBound[0].x, current_lane.leftBound[0].y])
        entrance_pt_right = np.array([current_lane.rightBound[0].x, current_lane.rightBound[0].y])
        exit_pt_left = np.array([current_lane.leftBound[-1].x, current_lane.leftBound[-1].y])
        exit_pt_right = np.array([current_lane.rightBound[-1].x, current_lane.rightBound[-1].y])

        lane_entrance = (entrance_pt_left + entrance_pt_right) / 2
        lane_exit = (exit_pt_left + exit_pt_right) / 2

        if np.linalg.norm(curr_waypoint - lane_exit) < np.linalg.norm(curr_waypoint - lane_entrance):
            left_bound = current_lane.leftBound
            right_bound = current_lane.rightBound
        else:
            temp = lane_entrance
            lane_entrance = lane_exit
            lane_exit = temp

            left_bound = np.array(list(reversed(current_lane.rightBound)))
            right_bound = np.array(list(reversed(current_lane.leftBound)))

        # 2)
        closest_left_way = []
        closest_right_way = []

        for node_idx in range(len(left_bound)-1):
            way_start = np.array([left_bound[node_idx].x, left_bound[node_idx].y])
            way_end = np.array([left_bound[node_idx+1].x, left_bound[node_idx+1].y])
            way_vec = way_end - way_start
            pos_to_way_start = pedestrian_pos - way_start

            t = np.dot(way_vec, pos_to_way_start) / np.dot(way_vec, way_vec)

            # ped can "drop a 90" to way if 0<=t<=1
            if t >= 0 and t <= 1:
                closest_left_way = way_vec

        for node_idx in range(len(right_bound)-1):
            way_start = np.array([right_bound[node_idx].x, right_bound[node_idx].y])
            way_end = np.array([right_bound[node_idx+1].x, right_bound[node_idx+1].y])
            way_vec = way_end - way_start
            pos_to_way_start = pedestrian_pos - way_start

            t = np.dot(way_vec, pos_to_way_start) / np.dot(way_vec, way_vec)

            if t >= 0 and t <= 1:
                closest_right_way = way_vec

        if len(closest_left_way) == 0:
            if np.linalg.norm(pedestrian_pos - lane_entrance) < np.linalg.norm(pedestrian_pos - lane_exit):
                closest_left_way = np.array([left_bound[0].x, left_bound[0].y])
            else:
                closest_left_way = np.array([left_bound[-1].x, left_bound[-1].y])

        if len(closest_right_way) == 0:
            if np.linalg.norm(pedestrian_pos - lane_entrance) < np.linalg.norm(pedestrian_pos - lane_exit):
                closest_right_way = np.array([right_bound[0].x, right_bound[0].y])
            else:
                closest_right_way = np.array([right_bound[-1].x, right_bound[-1].y])

        # 3)
        center_pt = (closest_left_way + closest_right_way) / 2
        return center_pt

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
