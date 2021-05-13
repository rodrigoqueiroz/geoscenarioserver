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
from util.Utils import get_lanelet_entry_exit_points
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

        self.lanelet_of_curr_waypoint = None
        self.inverted_path = False
        self.previous_maneuver = None


    def start(self):
        log.info('PLANNER PROCESS START for Pedestrian {}'.format(self.pid))
        #Behavior Layer
        #Note: If an alternative behavior module is to be used, it must be replaced here.
        self.behavior_model = BehaviorModels(self.pid, self.root_btree_name, self.btree_reconfig, self.btree_locations, self.btype)
        pedestrian_pos = np.array([self.sp.state.x, self.sp.state.y])
        self.plan_local_path(pedestrian_pos)
        self.lanelet_of_curr_waypoint = self.get_space_occupied_by_pedestrian(self.sp.current_waypoint)


    def plan_local_path(self, planning_position):
        occupied_spaces = self.lanelet_map.get_spaces_list_occupied_by_pedestrian(planning_position)

        # find which crosswalks are accessible from current lanelet or area
        paths_to_accessible_crosswalks = []

        for xwalk in self.sim_traffic.crosswalks:
            for ll in occupied_spaces['lanelets']:
                route_ll_to_xwalk = self.lanelet_map.routing_graph_pedestrians.getRoute(ll, xwalk)
                if route_ll_to_xwalk:
                    path_to_xwalk = self.lanelet_map.get_pedestrian_shortest_path(ll.id, xwalk.id)
                    # ensure there is only one crosswalk in path to target crosswalk (excluding current lanelet)
                    num_xwalks_in_path = sum([path_seg.attributes["subtype"] == "crosswalk" for path_seg in list(path_to_xwalk)])
                    if num_xwalks_in_path == 1:
                        paths_to_accessible_crosswalks.append(path_to_xwalk)
                else:
                    # check if there is a route from xwalk to lanelet (this case is necessary due to restrictions on the lanelet map)
                    route_xwalk_to_ll = self.lanelet_map.routing_graph_pedestrians.getRoute(xwalk, ll)
                    if route_xwalk_to_ll:
                        path_to_ll = self.lanelet_map.get_pedestrian_shortest_path(xwalk.id, ll.id)
                        # ensure there is only one crosswalk in path to target crosswalk (excluding current lanelet)
                        num_xwalks_in_path = sum([path_seg.attributes["subtype"] == "crosswalk" for path_seg in list(path_to_ll)])
                        if num_xwalks_in_path == 1:
                            paths_to_accessible_crosswalks.append(path_to_ll)


            for area in occupied_spaces['areas']:
                # crosswalk is accessible from area if they share a node
                if self.lanelet_map.area_and_lanelet_share_node(area, xwalk):
                    # determine entrance and exit points
                    entrance_pt, exit_pt = get_lanelet_entry_exit_points(xwalk)

                    if np.linalg.norm(exit_pt - planning_position) < np.linalg.norm(entrance_pt - planning_position):
                        # invert crosswalk way so that entrance is closer to ped than exit
                        xwalk = xwalk.invert()

                    paths_to_accessible_crosswalks.append([area, xwalk])

        # choose the path with the crosswalk that bring ped closest to destination
        chosen_path = None
        distance_to_dest = np.linalg.norm(self.sp.destination - planning_position)

        if len(paths_to_accessible_crosswalks) > 0:
            target_id = -1
            target_entry_pt = [None, None]
            target_exit_pt = [None, None]

            for path in paths_to_accessible_crosswalks:
                inverted_path = path[0].attributes['subtype'] == "crosswalk"

                if inverted_path:
                    xwalk = path[0]
                else:
                    xwalk = path[-1]

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
            if all([pt != None for pt in target_entry_pt]):
                waypoints = [target_entry_pt, target_exit_pt]
            else:
                waypoints = [self.sp.destination]
        else:
            # there are no accessible crosswalks
            waypoints = [self.sp.destination]

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
        else:
            self.sp.current_lanelet = self.get_space_occupied_by_pedestrian(pedestrian_pos)

        reg_elems = self.get_reg_elem_states(pedestrian_state, self.sp.current_lanelet)
        pedestrian_speed = {'default_desired': self.sp.default_desired_speed,
                            'current_desired': self.sp.curr_desired_speed}

        # Get planner state
        planner_state = PedestrianPlannerState(
                            pedestrian_state=pedestrian_state,
                            pedestrian_speed=pedestrian_speed,
                            path = self.sp.path,
                            waypoint = self.sp.current_waypoint,
                            target_crosswalk = self.sp.target_crosswalk,
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
                                                                                            self.lanelet_of_curr_waypoint,
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


    def get_reg_elem_states(self, pedestrian_state, current_lanelet):
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
