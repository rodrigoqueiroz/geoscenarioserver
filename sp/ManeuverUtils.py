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
from mapping.LaneletMap import LaneletMap
from util.Transformations import normalize
from util.Utils import get_lanelet_entry_exit_points, line_segments_intersect, orientation, point_in_rectangle


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
    if not planner_state.current_lanelet:
        return False
    return planner_state.current_lanelet.attributes['subtype'] == 'crosswalk'


def past_crosswalk_halfway(planner_state):
    P = np.array([planner_state.pedestrian_state.x, planner_state.pedestrian_state.y])
    crosswalk = planner_state.current_lanelet
    right = crosswalk.rightBound
    left = crosswalk.leftBound

    # get four points of exit half of xwalk in counterclockwise direction
    '''
    ----------------D----------------> C
                    |   exit half
                    |   of crosswalk
    ----------------A----------------> B
    '''
    B = np.array([right[-1].x, right[-1].y])
    A = (np.array([right[0].x, right[0].y]) + B) / 2
    C = np.array([left[-1].x, left[-1].y])
    D = (np.array([left[0].x, left[0].y]) + C) / 2

    return point_in_rectangle(P, A, B, C, D)


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


def approaching_crosswalk(planner_state):
    if planner_state.target_crosswalk["id"] == -1:
        return False

    threshold_dist = 3
    pedestrian_pos = np.array([planner_state.pedestrian_state.x, planner_state.pedestrian_state.y])

    if not planner_state.lanelet_map.inside_lanelet_or_area(pedestrian_pos, planner_state.current_lanelet):
        return False

    crosswalk_entry = planner_state.target_crosswalk["entry"]
    dist_to_crosswalk_entrance = np.linalg.norm(crosswalk_entry - pedestrian_pos)

    return dist_to_crosswalk_entrance < threshold_dist


def can_cross_before_red(planner_state):
    """
    return ((distance to entry) + (distance from entry to exit)) / (default desired speed) < (time to red)
    """
    pedestrian_pos = np.array([planner_state.pedestrian_state.x, planner_state.pedestrian_state.y])
    crosswalk_entry = planner_state.target_crosswalk["entry"]
    crosswalk_exit = planner_state.target_crosswalk["exit"]

    dist_pos_to_entry = np.linalg.norm(crosswalk_entry - pedestrian_pos)
    dist_entry_to_exit = np.linalg.norm(crosswalk_exit - crosswalk_entry)

    crossing_possible = (dist_pos_to_entry + dist_entry_to_exit) / planner_state.pedestrian_speed["default_desired"] < planner_state.crossing_light_time_to_red

    return crossing_possible
