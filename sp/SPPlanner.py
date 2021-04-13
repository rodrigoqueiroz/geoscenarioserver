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
import lanelet2.core
from mapping.LaneletMap import *
from Actor import *
from SimTraffic import *

import time

class SPPlanner(object):
    def __init__(self, sp, sim_traffic, btree_locations):
        self.pid = int(sp.id)
        self.laneletmap:LaneletMap = sim_traffic.lanelet_map
        self.sim_config = sim_traffic.sim_config
        self.sim_traffic:SimTraffic = sim_traffic
        #Subprocess space
        # Reference path that the planner will use for all transformations and planning

        self.root_btree_name = sp.root_btree_name
        self.btree_reconfig = sp.btree_reconfig
        self.behavior_model = None
        self.mconfig = None
        self.btree_locations = btree_locations
        self.btype = sp.btype

        self.route = []
        self.curr_route_node = sp.curr_route_node
        self.current_desired_speed = sp.current_desired_speed
        self.replan_route = True


    def start(self):
        log.info('PLANNER PROCESS START for Pedestrian {}'.format(self.pid))
        #Behavior Layer
        #Note: If an alternative behavior module is to be used, it must be replaced here.
        self.behavior_model = BehaviorModels(self.pid, self.root_btree_name, self.btree_reconfig, self.btree_locations, self.btype)
        self.plan_route()


    def plan_route(self):
        pedestrian_state = self.sim_traffic.pedestrians[self.pid].state

        ped_pos = np.array([pedestrian_state.x, pedestrian_state.y])
        destination = np.array(self.sim_config.pedestrian_goal_points[self.pid][-1])
        dist_to_dest = np.linalg.norm(destination - ped_pos)

        self.route = [ped_pos]
        xwalks_in_plan = []
        found_closest_exit = False

        while not found_closest_exit:
            closest_xwalk = -1
            xwalks_not_in_plan = {xwalk_id: entry_pts for (xwalk_id, entry_pts) in self.sim_traffic.crosswalk_entry_pts.items() if xwalk_id not in xwalks_in_plan}
            closest_entry_dist = np.linalg.norm(list(xwalks_not_in_plan.values())[0] - self.route[-1])
            dist_to_dest = np.linalg.norm(destination - self.route[-1])

            for xwalk_id,entry_pts in xwalks_not_in_plan.items():
                # determine entrance and exit by distance to pedestrian
                new_entrance = entry_pts[0]
                new_exit = entry_pts[1]
                if np.linalg.norm(new_exit - self.route[-1]) < np.linalg.norm(new_entrance - self.route[-1]):
                    new_entrance = entry_pts[1]
                    new_exit = entry_pts[0]

                dist_to_entrance = np.linalg.norm(self.route[-1] - new_entrance)
                dist_from_exit_to_dest = np.linalg.norm(destination - new_exit)

                # pick the closest entrance with an exit that is closer to dest than the current last route node
                if dist_to_entrance < closest_entry_dist and dist_from_exit_to_dest < dist_to_dest:
                    entrance = new_entrance
                    exit = new_exit
                    closest_entry_dist = dist_to_entrance
                    closest_xwalk = xwalk_id

            if closest_xwalk != -1:
                xwalks_in_plan.append(closest_xwalk)
                self.route.append(entrance)
                self.route.append(exit)
            else:
                found_closest_exit = True

        self.route.append(destination)
        self.route = self.route[1:] # remove current ped position from route


    def run_planner(self):
        pedestrian_state = self.sim_traffic.pedestrians[self.pid].state

        reg_elems = self.get_reg_elem_states(pedestrian_state)
        pedestrian_speed = {'default_desired': self.sim_traffic.pedestrians[self.pid].default_desired_speed,
                            'current_desired': self.sim_traffic.pedestrians[self.pid].current_desired_speed}

        # Get planner state
        planner_state = PedestrianPlannerState(
                            pedestrian_state=pedestrian_state,
                            pedestrian_speed=pedestrian_speed,
                            route=self.route,
                            curr_route_node=self.curr_route_node,
                            traffic_vehicles=self.sim_traffic.vehicles,
                            regulatory_elements=reg_elems,
                            pedestrians=self.sim_traffic.pedestrians,
                            lanelet_map=self.sim_traffic.lanelet_map
                        )

        # BTree Tick
        mconfig, snapshot_tree = self.behavior_model.tick(planner_state)

        # new maneuver
        if self.mconfig and self.mconfig.mkey != mconfig.mkey:
            log.info("PID {} started maneuver {}".format(self.pid, mconfig.mkey.name))
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
            self.curr_route_node, speed_chg = plan_maneuver(mconfig.mkey,
                                                            mconfig,
                                                            planner_state.pedestrian_state,
                                                            planner_state.pedestrian_speed,
                                                            planner_state.route,
                                                            planner_state.curr_route_node,
                                                            planner_state.traffic_vehicles,
                                                            planner_state.pedestrians)

            if speed_chg != None:
                self.current_desired_speed = speed_chg


    def get_space_occupied_by_pedestrian(self, pedestrian_state):
        # check if ped on lanelet
        ll = self.sim_traffic.lanelet_map.get_occupying_lanelet_by_participant(pedestrian_state.x, pedestrian_state.y, "pedestrian")
        if ll != None:
            return ll

        # check if ped on area
        area = self.sim_traffic.lanelet_map.get_occupying_area(pedestrian_state.x, pedestrian_state.y)
        if area != None:
            return area

        # otherwise, ped is off the map
        return None


    def get_reg_elem_states(self, pedestrian_state):
        reg_elem_states = []

        traffic_light_states = {}
        for lid, tl in self.sim_traffic.traffic_lights.items():
            traffic_light_states[lid] = tl.current_color.value

        curr_ll_or_area = self.get_space_occupied_by_pedestrian(pedestrian_state)

        if not curr_ll_or_area:
            return reg_elem_states

        # Get regulatory elements acting on this lanelet
        reg_elems = curr_ll_or_area.regulatoryElements


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
