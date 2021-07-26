#!/usr/bin/env python
#rqueiroz@uwaterloo.ca
#d43sharm@uwaterloo.ca
#slarter@uwaterloo.ca
# --------------------------------------------
# GEOSCENARIO SIMULATION PEDESTRIAN PLANNER
# --------------------------------------------

import datetime
import numpy as np
from multiprocessing import shared_memory, Process, Lock, Array, Manager
from typing import Dict, List
import glog as log
from copy import copy
from TickSync import TickSync
from mapping.LaneletMap import LaneletMap
from sp.ManeuverConfig import *
from sp.ManeuverModels import plan_maneuver
from sp.btree.BehaviorModels import BehaviorModels
from sp.SPPlannerState import PedestrianPlannerState, TrafficLightState
from util.Utils import get_lanelet_entry_exit_points, angle_btwn_vectors
from TrafficLight import TrafficLightColor
import lanelet2.core
from mapping.LaneletMap import *
from Actor import *
from SimTraffic import *

import time

class SPPlanner(object):
    def __init__(self, sp, sim_traffic, btree_locations):
        self.pid = int(sp.id)
        self.lanelet_map:LaneletMap = sim_traffic.lanelet_map
        self.sim_config = sim_traffic.sim_config
        self.sim_traffic:SimTraffic = sim_traffic

        self.sp = sp
        self.root_btree_name = sp.root_btree_name
        self.btree_reconfig = sp.btree_reconfig
        self.behavior_model = None
        self.mconfig = None
        self.btree_locations = btree_locations
        self.btype = sp.btype

        self.inverted_path = False
        self.previous_maneuver = None
        self.selected_target_crosswalk = False
        self.planning_position = []


    def start(self):
        log.info('PLANNER PROCESS START for Pedestrian {}'.format(self.pid))
        #Behavior Layer
        #Note: If an alternative behavior module is to be used, it must be replaced here.
        self.behavior_model = BehaviorModels(self.pid, self.root_btree_name, self.btree_reconfig, self.btree_locations, self.btype)
        pedestrian_pos = np.array([self.sp.state.x, self.sp.state.y])
        self.plan_local_path(pedestrian_pos, False)

    def plan_local_path(self, planning_position, consider_light_states, aggressiveness_level = 1):
        # get list of lanelets and spaces containing the planning position
        occupied_spaces = self.lanelet_map.get_spaces_list_occupied_by_pedestrian(planning_position)

        ''' Find which crosswalks are accessible from current lanelet or area '''
        paths_to_accessible_crosswalks = []

        for xwalk in self.sim_traffic.crosswalks:
            for ll in occupied_spaces['lanelets']:
                # check if there is a route from lanelet to xwalk
                route_ll_to_xwalk = self.lanelet_map.routing_graph_pedestrians.getRoute(ll, xwalk)
                if route_ll_to_xwalk:
                    path_to_xwalk = self.lanelet_map.get_pedestrian_shortest_path(ll.id, xwalk.id)
                    # ensure there is only one crosswalk in path to target crosswalk (excluding current lanelet)
                    num_xwalks_in_path = sum([path_seg.attributes["subtype"] == "crosswalk" for path_seg in list(path_to_xwalk)])
                    if num_xwalks_in_path == 1:
                        paths_to_accessible_crosswalks.append(path_to_xwalk)
                else:
                    # check if there is a route from xwalk to lanelet
                    # (this case is necessary due to lanelet direction restrictions on the lanelet map)
                    route_xwalk_to_ll = self.lanelet_map.routing_graph_pedestrians.getRoute(xwalk, ll)
                    if route_xwalk_to_ll:
                        path_to_ll = self.lanelet_map.get_pedestrian_shortest_path(xwalk.id, ll.id)
                        # ensure there is only one crosswalk in path from target crosswalk (excluding current lanelet)
                        num_xwalks_in_path = sum([path_seg.attributes["subtype"] == "crosswalk" for path_seg in list(path_to_ll)])
                        if num_xwalks_in_path == 1:
                            paths_to_accessible_crosswalks.append(path_to_ll)

            for area in occupied_spaces['areas']:
                # crosswalk is accessible from area if they share a node
                if self.lanelet_map.area_and_lanelet_share_node(area, xwalk):
                    entrance_pt, exit_pt = get_lanelet_entry_exit_points(xwalk)

                    if np.linalg.norm(exit_pt - planning_position) < np.linalg.norm(entrance_pt - planning_position):
                        # invert crosswalk way so that entrance is closer to ped than exit
                        xwalk = xwalk.invert()

                    paths_to_accessible_crosswalks.append([area, xwalk])

        ''' Choose the path with the crosswalk that bring ped closest to destination '''
        chosen_path = []
        distance_to_dest = np.linalg.norm(self.sp.destination - planning_position)

        if len(paths_to_accessible_crosswalks) > 0:
            target_id = -1
            target_entry_pt = []
            target_exit_pt = []

            if consider_light_states:
                all_red_lights = True
                self.selected_target_crosswalk = False
                best_candidate_can_cross = {'id': -1,
                                            'entry': [],
                                            'exit': [],
                                            'exit_to_dest_dist': np.linalg.norm(self.sp.destination - planning_position),
                                            'path': [],
                                            'inverted_path': False,
                                            'color': None,
                                            'ttr': []}
                best_candidate_must_wait = {'id': -1,
                                            'entry': [],
                                            'exit': [],
                                            'exit_to_dest_dist': np.linalg.norm(self.sp.destination - planning_position),
                                            'path': [],
                                            'inverted_path': False,
                                            'color': None,
                                            'ttr': []}
                num_candidates = 0

                for path in paths_to_accessible_crosswalks:
                    # inverted_path is True if there was a valid route found from a crosswalk to the planning position but not the other way
                    inverted_path = (path[0].attributes['subtype'] == "crosswalk")
                    xwalk = path[0] if inverted_path else path[-1]
                    entrance_pt, exit_pt = get_lanelet_entry_exit_points(xwalk)

                    if inverted_path:
                        temp = entrance_pt
                        entrance_pt = exit_pt
                        exit_pt = temp

                    crossing_light_color, crossing_light_ttr = self.get_crossing_light_state(xwalk)

                    # check if candidate
                    exit_to_dest_dist = np.linalg.norm(self.sp.destination - exit_pt)
                    entrance_to_dest_dist = np.linalg.norm(self.sp.destination - entrance_pt)
                    if exit_to_dest_dist < entrance_to_dest_dist:
                        candidate = {   'id': xwalk.id,
                                        'entry': entrance_pt,
                                        'exit': exit_pt,
                                        'exit_to_dest_dist': exit_to_dest_dist,
                                        'path': path,
                                        'inverted_path': inverted_path,
                                        'color': crossing_light_color,
                                        'ttr': crossing_light_ttr   }
                        num_candidates += 1

                        if crossing_light_color == TrafficLightColor.Green or not crossing_light_color:
                            all_red_lights = False
                            if exit_to_dest_dist < best_candidate_can_cross['exit_to_dest_dist']:
                                best_candidate_can_cross = candidate

                        elif crossing_light_color == TrafficLightColor.Yellow:
                            all_red_lights = False
                            better_can_cross = exit_to_dest_dist < best_candidate_can_cross['exit_to_dest_dist']
                            better_must_wait = exit_to_dest_dist < best_candidate_must_wait['exit_to_dest_dist']

                            if aggressiveness_level == 1:
                                if better_must_wait:
                                    best_candidate_must_wait = candidate
                            elif aggressiveness_level == 3:
                                # TODO: add condition that checks if vehicles are coming
                                if better_can_cross:
                                    best_candidate_can_cross = candidate
                            else:
                                dist_pos_to_entry = np.linalg.norm(entrance_pt - planning_position)
                                dist_entry_to_exit = np.linalg.norm(exit_pt - entrance_pt)
                                if better_can_cross and (dist_pos_to_entry + dist_entry_to_exit) / self.sp.default_desired_speed < crossing_light_ttr:
                                    best_candidate_can_cross = candidate
                                elif better_must_wait:
                                    best_candidate_must_wait = candidate

                        elif crossing_light_color == TrafficLightColor.Red:
                            if aggressiveness_level == 3 and exit_to_dest_dist < best_candidate_can_cross['exit_to_dest_dist']:
                                best_candidate_can_cross = candidate
                            elif exit_to_dest_dist < best_candidate_must_wait['exit_to_dest_dist']:
                                best_candidate_must_wait = candidate

                if best_candidate_can_cross['id'] != -1:
                    chosen_path = best_candidate_can_cross['path']
                    target_id = best_candidate_can_cross['id']
                    target_entry_pt = best_candidate_can_cross['entry']
                    target_exit_pt = best_candidate_can_cross['exit']
                    self.inverted_path = best_candidate_can_cross['inverted_path']
                    self.selected_target_crosswalk = True
                elif best_candidate_must_wait['id'] != -1:
                    chosen_path = best_candidate_must_wait['path']
                    target_id = best_candidate_must_wait['id']
                    target_entry_pt = best_candidate_must_wait['entry']
                    target_exit_pt = best_candidate_must_wait['exit']
                    self.inverted_path = best_candidate_must_wait['inverted_path']
                    self.selected_target_crosswalk = True
                elif num_candidates == 1:
                    chosen_path = candidate['path']
                    target_id = candidate['id']
                    target_entry_pt = candidate['entry']
                    target_exit_pt = candidate['exit']
                    self.inverted_path = candidate['inverted_path']
                    self.selected_target_crosswalk = True

                if all_red_lights and num_candidates > 1:
                    self.selected_target_crosswalk = False
            else:
                for path in paths_to_accessible_crosswalks:
                    # inverted_path is True if there was a valid route found from a crosswalk to the planning position
                    inverted_path = (path[0].attributes['subtype'] == "crosswalk")

                    xwalk = path[0] if inverted_path else path[-1]

                    entrance_pt, exit_pt = get_lanelet_entry_exit_points(xwalk)

                    if inverted_path:
                        temp = entrance_pt
                        entrance_pt = exit_pt
                        exit_pt = temp

                    # update selected path if this crosswalk brings ped closer to destination
                    exit_to_dest_dist = np.linalg.norm(self.sp.destination - exit_pt)
                    entrance_to_dest_dist = np.linalg.norm(self.sp.destination - entrance_pt)
                    if (exit_to_dest_dist < entrance_to_dest_dist) and (exit_to_dest_dist < distance_to_dest):
                        distance_to_dest = exit_to_dest_dist
                        chosen_path = path
                        target_id = xwalk.id
                        target_entry_pt = entrance_pt
                        target_exit_pt = exit_pt
                        self.inverted_path = inverted_path

            # set entry and exit points if there is a useful crosswalk
            if consider_light_states and not self.selected_target_crosswalk:
                waypoints = [planning_position]
            elif target_id != -1:
                waypoints = [target_entry_pt, target_exit_pt]
            else:
                waypoints = [self.sp.destination]
        else:
            # there are no accessible crosswalks
            waypoints = [self.sp.destination]

        ''' If no path containing a crosswalk chosen, construct path from sequence of consecutive lanelets '''
        if len(chosen_path) == 0 and not consider_light_states:
            invert_ll = False
            invert_candidate = False
            spaces_of_dest = self.lanelet_map.get_spaces_list_occupied_by_pedestrian(self.sp.destination)
            spaces_of_dest_ids = [ll.id for ll in spaces_of_dest['lanelets']]

            # list of lanelets containing both the destination and planning position
            curr_lls_containing_dest = [ll for ll in occupied_spaces['lanelets'] if ll.id in spaces_of_dest_ids]

            if len(curr_lls_containing_dest) > 0:
                # current lanelet contains the ped's destination
                selected_ll = curr_lls_containing_dest[0]

                # invert the lanelet if direction towards destination is heading to the lanelet entrance
                entrance_pt, exit_pt = get_lanelet_entry_exit_points(selected_ll)
                pos_dest_vec = self.sp.destination - planning_position
                pos_entry_vec = entrance_pt - planning_position
                pos_exit_vec = exit_pt - planning_position
                if angle_btwn_vectors(pos_dest_vec, pos_entry_vec) < angle_btwn_vectors(pos_dest_vec, pos_exit_vec):
                    selected_ll = selected_ll.invert()

                chosen_path.append(selected_ll)
            else:
                ''' construct sequence of lanelets from planning position to destination by
                    considering all previous and following lanelets at each step and selecting
                    one that brings ped closest to destination
                '''

                # current lanelet does not contain the ped's destination
                selected_ll = occupied_spaces['lanelets'][0]
                selected_entrance_pt, selected_exit_pt = get_lanelet_entry_exit_points(selected_ll)

                if np.linalg.norm(self.sp.destination - selected_entrance_pt) < np.linalg.norm(self.sp.destination - selected_exit_pt):
                    selected_exit_pt = selected_entrance_pt
                    invert_candidate = True

                for ll in occupied_spaces['lanelets'][1:]:
                    entrance_pt, exit_pt = get_lanelet_entry_exit_points(ll)

                    if np.linalg.norm(self.sp.destination - exit_pt) < np.linalg.norm(self.sp.destination - selected_exit_pt):
                        selected_exit_pt = exit_pt
                        selected_ll = ll
                        invert_candidate = False

                    if np.linalg.norm(self.sp.destination - entrance_pt) < np.linalg.norm(self.sp.destination - selected_exit_pt):
                        selected_exit_pt = entrance_pt
                        selected_ll = ll
                        invert_candidate = True

                invert_ll = invert_candidate

                if invert_ll:
                    selected_ll = selected_ll.invert()

                chosen_path.append(selected_ll)

                previous_lls = self.lanelet_map.routing_graph_pedestrians.previous(selected_ll)
                following_lls = self.lanelet_map.routing_graph_pedestrians.following(selected_ll)

                next_lls_containing_dest = [ll for ll in (previous_lls + following_lls) if ll.id in spaces_of_dest_ids]

                # build path of connected previous/following lanelets until lanelet containing destination is next
                while len(next_lls_containing_dest) == 0:
                    invert_ll = False

                    candidate_ll = None
                    closest_dist_to_dest = np.linalg.norm(self.sp.destination - selected_exit_pt)

                    for prev_or_follow_ll in (previous_lls + following_lls):
                        invert_candidate = False
                        entrance_pt, exit_pt = get_lanelet_entry_exit_points(prev_or_follow_ll)
                        if np.linalg.norm(exit_pt - selected_exit_pt) < 0.005:
                            exit_pt = entrance_pt
                            invert_candidate = True

                        dist_exit_to_dest = np.linalg.norm(self.sp.destination - exit_pt)
                        if dist_exit_to_dest < closest_dist_to_dest:
                            candidate_ll = prev_or_follow_ll
                            candidate_exit_pt = exit_pt
                            closest_dist_to_dest = dist_exit_to_dest
                            invert_ll = invert_candidate

                    if not candidate_ll:
                        break

                    selected_ll = candidate_ll
                    selected_exit_pt = candidate_exit_pt
                    if invert_ll:
                        selected_ll = selected_ll.invert()
                    chosen_path.append(selected_ll)

                    previous_lls = self.lanelet_map.routing_graph_pedestrians.previous(selected_ll)
                    following_lls = self.lanelet_map.routing_graph_pedestrians.following(selected_ll)
                    spaces_of_dest_ids = [ll.id for ll in spaces_of_dest['lanelets']]
                    next_lls_containing_dest = [ll for ll in (previous_lls + following_lls) if ll.id in spaces_of_dest_ids]

                chosen_path.append(next_lls_containing_dest[0])

        self.sp.path = chosen_path
        self.sp.waypoints = iter(waypoints)
        self.sp.current_waypoint = next(self.sp.waypoints)
        self.sp.target_crosswalk = {'id': target_id, 'entry': target_entry_pt, 'exit': target_exit_pt}


    def run_planner(self, sim_time):
        pedestrian_state = self.sp.state
        pedestrian_pos = np.array([pedestrian_state.x, pedestrian_state.y])

        # get current lanelet/area
        if self.sp.path:
            for ll_or_area in self.sp.path:
                if self.lanelet_map.inside_lanelet_or_area(pedestrian_pos, ll_or_area):
                    self.sp.current_lanelet = ll_or_area
                    break
        else:
            self.sp.current_lanelet = self.get_space_occupied_by_pedestrian(pedestrian_pos)

        reg_elems = self.get_reg_elem_states(self.sp.current_lanelet)

        crossing_light_color = None
        crossing_light_ttr = np.inf
        if self.sp.target_crosswalk['id'] != -1:
            if self.inverted_path:
                crosswalk = self.sp.path[0]
            else:
                crosswalk = self.sp.path[-1]

            crossing_light_color, crossing_light_ttr = self.get_crossing_light_state(crosswalk)

        pedestrian_speed = {'default_desired': self.sp.default_desired_speed,
                            'current_desired': self.sp.curr_desired_speed}

        # Get planner state
        planner_state = PedestrianPlannerState(
                            pedestrian_state=pedestrian_state,
                            pedestrian_speed=pedestrian_speed,
                            path = self.sp.path,
                            waypoint = self.sp.current_waypoint,
                            target_crosswalk = self.sp.target_crosswalk,
                            selected_target_crosswalk = self.selected_target_crosswalk,
                            current_lanelet = self.sp.current_lanelet,
                            crossing_light_color = crossing_light_color,
                            crossing_light_time_to_red = crossing_light_ttr,
                            destination = np.array(self.sim_config.pedestrian_goal_points[self.pid][-1]),
                            traffic_vehicles=self.sim_traffic.vehicles,
                            regulatory_elements=reg_elems,
                            pedestrians=self.sim_traffic.pedestrians,
                            lanelet_map=self.sim_traffic.lanelet_map,
                            previous_maneuver = self.previous_maneuver
                        )

        # BTree Tick
        mconfig, snapshot_tree = self.behavior_model.tick(planner_state)

        # new maneuver
        if self.mconfig and self.mconfig.mkey != mconfig.mkey:
            log.info("PID {} started maneuver {}".format(self.pid, mconfig.mkey.name))
            self.sp.maneuver_sequence.append([sim_time, mconfig.mkey.name])
            # print sp state and deltas
            state_str = (
                "PID {}:\n"
                "   position    sim=({:.3f},{:.3f})\n"
                "   speed       {:.3f}\n"
            ).format(
                self.pid,
                pedestrian_state.x, pedestrian_state.y,
                np.linalg.norm([pedestrian_state.x_vel, pedestrian_state.y_vel])
            )
            log.info(state_str)
        self.mconfig = mconfig

        # Maneuver tick
        if mconfig:
            #replan maneuver
            self.sp.direction, self.sp.current_waypoint, self.sp.curr_desired_speed = plan_maneuver(mconfig.mkey,
                                                                                            mconfig,
                                                                                            self.sp,
                                                                                            planner_state.pedestrian_state,
                                                                                            planner_state.pedestrian_speed,
                                                                                            planner_state.target_crosswalk,
                                                                                            planner_state.traffic_vehicles,
                                                                                            planner_state.pedestrians,
                                                                                            planner_state.previous_maneuver)
            self.previous_maneuver = mconfig.mkey

    def stop(self):
        pass
        # write manuever sequences to csv file
        # with open('sp/maneuver_sequences/pid_{}.csv'.format(self.pid), 'w', newline='') as maneuvers_file:
        #     maneuver_writer = csv.writer(maneuvers_file, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
        #     maneuver_writer.writerow(["sim time", "maneuver"])
        #
        #     for maneuver_row in self.sp.maneuver_sequence:
        #         maneuver_writer.writerow(maneuver_row)


    def get_space_occupied_by_pedestrian(self, position):
        # check if ped on lanelet
        ll = self.sim_traffic.lanelet_map.get_occupying_lanelet_by_participant(position[0], position[1], "pedestrian")
        if ll != None:
            return ll

        # check if ped on area
        area = self.sim_traffic.lanelet_map.get_occupying_area(position[0], position[1])
        if area != None:
            return area

        # otherwise, ped is off the map
        return None


    def get_reg_elem_states(self, current_lanelet):
        reg_elem_states = []

        if not current_lanelet:
            return reg_elem_states

        traffic_light_states = {}
        for lid, tl in self.sim_traffic.traffic_lights.items():
            traffic_light_states[lid] = tl.current_color.value

        # Get regulatory elements acting on this lanelet
        reg_elems = current_lanelet.regulatoryElements


        for re in reg_elems:
            if isinstance(re, lanelet2.core.TrafficLight):
                # lanelet2 traffic lights must have a corresponding state from the main process
                if re.id not in traffic_light_states:
                    continue

                stop_linestring = re.parameters['ref_line']

                # stop at middle of stop line
                stop_pos = (np.asarray([stop_linestring[0][0].x, stop_linestring[0][0].y]) + np.asarray([stop_linestring[0][-1].x, stop_linestring[0][-1].y])) / 2
                reg_elem_states.append(TrafficLightState(color=traffic_light_states[re.id], stop_position=stop_pos))

        return reg_elem_states


    def get_crossing_light_state(self, crosswalk):
        if not crosswalk:
            return None, np.inf

        traffic_light_states = {}
        traffic_light_ttr = {}
        for lid, tl in self.sim_traffic.traffic_lights.items():
            traffic_light_states[lid] = tl.current_color.value
            traffic_light_ttr[lid] = tl.time_to_red

        # Get regulatory elements acting on this lanelet
        reg_elems = crosswalk.regulatoryElements

        for re in reg_elems:
            if isinstance(re, lanelet2.core.TrafficLight):
                # lanelet2 traffic lights must have a corresponding state from the main process
                if re.id not in traffic_light_states:
                    continue

                return traffic_light_states[re.id], traffic_light_ttr[re.id]

        return None, np.inf
