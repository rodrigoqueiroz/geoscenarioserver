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
from util.Utils import get_lanelet_entry_exit_points, line_segments_intersect, orientation


def has_reached_point(pedestrian_state, point, threshold=1.5):
    """ Checks if the pedestrian has reached a given point in the cartesian frame
        @param point: Array [x,y] node position
        @param threshold: Max acceptable distance to point
    """
    pedestrian_pos = np.array([pedestrian_state.x, pedestrian_state.y])

    return np.linalg.norm(np.asarray(point) - pedestrian_pos) < threshold


def dir_to_follow_lane_border(pedestrian_state, current_lane, curr_waypoint, inverted_path):
    ''' find unit vector of desired direction to follow left or right border
    '''
    pedestrian_pos = np.array([pedestrian_state.x, pedestrian_state.y])
    dist_from_border = 0.75

    if inverted_path:
        current_lane = current_lane.invert()

    # find entrance and exit points of the current lanelet
    entrance_pt, exit_pt = get_lanelet_entry_exit_points(current_lane)

    # determine which border (left or right) ped should follow from orientation of waypoint
    waypt_orientation = orientation(pedestrian_pos, exit_pt, curr_waypoint)

    left_end_pt = np.array([current_lane.leftBound[-1].x, current_lane.leftBound[-1].y])
    right_end_pt = np.array([current_lane.rightBound[-1].x, current_lane.rightBound[-1].y])

    if waypt_orientation == 2:
        # waypoint on left side of line between position and lanelet exit
        follow_border = current_lane.leftBound
        # extra node is necessary to ensure ped can actually exit the lanelet
        extra_node = left_end_pt + (normalize(right_end_pt - left_end_pt)*dist_from_border)
    else:
        follow_border = current_lane.rightBound
        extra_node = right_end_pt + (normalize(left_end_pt - right_end_pt)*dist_from_border)

    # check if extra node is in line of sight
    if has_line_of_sight_to_point(pedestrian_pos, extra_node, current_lane):
        return normalize(extra_node - pedestrian_pos)

    # traverse follow_border segments in reverse order to find furthest target point with direct line of sight
    for node_idx in range(len(follow_border)-1, 0, -1):
        seg_start_pt = np.array([follow_border[node_idx-1].x, follow_border[node_idx-1].y])
        seg_end_pt = np.array([follow_border[node_idx].x, follow_border[node_idx].y])

        seg_vec = seg_end_pt - seg_start_pt

        if waypt_orientation == 2:
            # target point to right of left border
            seg_norm = normalize(np.array([seg_vec[1], -seg_vec[0]]))
        else:
            # target point to left of right border
            seg_norm = normalize(np.array([-seg_vec[1], seg_vec[0]]))

        # get point 0.5m in from end of segment
        target_pt = seg_end_pt + (seg_norm*dist_from_border)

        if has_line_of_sight_to_point(pedestrian_pos, target_pt, current_lane):
            return normalize(target_pt - pedestrian_pos)

    return normalize(curr_waypoint - pedestrian_pos)


def in_crosswalk_area(planner_state):
    cur_ll = planner_state.lanelet_map.get_occupying_lanelet_by_participant(planner_state.pedestrian_state.x, planner_state.pedestrian_state.y, "pedestrian")
    if cur_ll == None:
        return False
    return cur_ll.attributes["subtype"] == "crosswalk"


def has_line_of_sight_to_point(position, point, lanelet):
    ''' return True if there is a line of sight
        from stop_position to point in lanelet
    '''

    sight_line = [point, position]

    # check if left bound of lanelet obstructs line of sight to point
    for node_idx in range(len(lanelet.leftBound) - 1):
        p1 = [lanelet.leftBound[node_idx].x, lanelet.leftBound[node_idx].y]
        p2 = [lanelet.leftBound[node_idx + 1].x, lanelet.leftBound[node_idx + 1].y]

        seg = [p1, p2]

        if line_segments_intersect(seg, sight_line):
            return False

    # check if right bound of lanelet obstructs line of sight to point
    for node_idx in range(len(lanelet.rightBound) - 1):
        p1 = [lanelet.rightBound[node_idx].x, lanelet.rightBound[node_idx].y]
        p2 = [lanelet.rightBound[node_idx + 1].x, lanelet.rightBound[node_idx + 1].y]

        seg = [p1, p2]

        if line_segments_intersect(seg, sight_line):
            return False

    return True

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
