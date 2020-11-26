#!/usr/bin/env python
#rqueiroz@uwaterloo.ca
#d43sharm@uwaterloo.ca
# --------------------------------------------
# GEOSCENARIO SIMULATION VEHICLE PLANNER
# --------------------------------------------

import numpy as np
import random
import datetime
import time
from dataclasses import dataclass
from multiprocessing import shared_memory, Process, Lock, Array, Manager
from typing import Tuple, Dict, List
import glog as log

from TickSync import TickSync
from mapping.LaneletMap import LaneletMap
from sv.VehicleState import *
from sv.ManeuverConfig import *
from sv.ManeuverModels import *
from sv.btree.BTreeModel import * # Deprecated
#from sv.btree.BTreeFactory import *
from sv.btree.BehaviorModels import *
# import sv.SV
from util.Transformations import sim_to_frenet_frame, sim_to_frenet_position


@dataclass
class PlannerState:
    sim_time: float
    vehicle_state: VehicleState
    lane_config: LaneConfig
    traffic_vehicles: Dict
    pedestrians: List
    obstacles: List
    goal_point: Tuple[float,float] = None
    goal_point_frenet: Tuple[float,float] = None


class SVPlanner(object):
    def __init__(self, vid, btree_root, nvehicles, laneletmap, sim_config, traffic_state_sharr, debug_shdata):
        #MainProcess space:
        self._process = None
        self._traffic_state_sharr = traffic_state_sharr
        self._debug_shdata = debug_shdata
        self._mplan_sharr = None
        self.motion_plan = None
        #Shared space
        self.vid = int(vid)
        self.nvehicles = nvehicles
        self.laneletmap = laneletmap
        self.sim_config = sim_config
        self.lookahead_dist = 10

        #Subprocess space
        # Reference path that the planner will use for all transformations and planning
        self.reference_path = None
        self.btree_root = btree_root
        self.btree_model = None

    def start(self):
        #Create Shared arrray for Plan
        c = MotionPlan.VECTORSIZE
        self._mplan_sharr = Array('f', c)
        #Process based
        self._process = Process(target=self.run_planner_process, args=(self._traffic_state_sharr, self._mplan_sharr, self._debug_shdata), daemon=True)
        self._process.start()

    def stop(self):
        if self._process:
            log.info("Terminate Planner Process - vehicle {}".format(self.vid))
            self._process.terminate()

    def get_plan(self):
        # TODO: knowledge of reference path changing should be written even if trajectory is invalid
        # because then NEXT tick planner will write trajectory based on new path while SV is following
        # the old path. This could be solved by adding a 'frame' variable to the shared array, like
        # a sim frame position that can be used to compute the reference path when the SV notices it's
        # changed. Unlike `new_frenet_frame` this won't be a per-tick variable.
        plan = MotionPlan()
        self._mplan_sharr.acquire() #<=========LOCK
        plan.set_plan_vector(self._mplan_sharr[:])
        self._mplan_sharr.release() #<=========RELEASE
        #if (plan.t == 0): #if not valid
        #    return None
        if (self.motion_plan): #if same as current
            if (np.array_equal(self.motion_plan.get_plan_vector(), plan.get_plan_vector())):
                return None
        #Valid and New:
        self.motion_plan = plan
        return self.motion_plan

    #==SUB PROCESS=============================================

    def run_planner_process(self, traffic_state_sharr, mplan_sharr, debug_shdata):
        log.info('PLANNER PROCESS START for Vehicle {}'.format(self.vid))

        sync_planner = TickSync(rate=PLANNER_RATE, realtime=True, block=True, verbose=False, label="PP")

        #self.btree_model = BTreeModel(self.vid, self.btree_root)
        #self.btree_model = BTreeFactory(self.vid, self.btree_root).build_tree()
        self.btree_model = BehaviorModels(self.vid, self.btree_root)

        while sync_planner.tick():
            header, vehicle_state, traffic_vehicles = self.read_traffic_state(traffic_state_sharr)
            state_time = header[2]

            if self.reference_path is None:
                self.reference_path = self.laneletmap.get_global_path_for_route(
                    self.sim_config.lanelet_routes[self.vid], vehicle_state.x, vehicle_state.y)

            # Get traffic and lane config in current frenet frame
            planner_state = self.get_planner_state(sync_planner, vehicle_state, traffic_vehicles)
            if not planner_state:
                log.warn("Invalid planner state, skipping planning step...")
                continue

            #BTree Tick - using frenet state and lane config based on old ref path
            mconfig, ref_path_changed, snapshot_tree = self.btree_model.tick(planner_state)

            # when ref path changes, must recalculate the path, lane config and relative state of other vehicles
            if ref_path_changed:
                log.info("PATH CHANGED")

                self.reference_path = self.laneletmap.get_global_path_for_route(
                    self.sim_config.lanelet_routes[self.vid], vehicle_state.x, vehicle_state.y)

                # Regenerate planner state and tick btree again. Discard whether ref path changed again.
                planner_state = self.get_planner_state(sync_planner, vehicle_state, traffic_vehicles)
                if not planner_state:
                    log.warn("Invalid planner state, skipping planning step...")
                    continue
                mconfig, _, snapshot_tree = self.btree_model.tick(planner_state)

            #log.info('Plan {} at time {} and FRENET STATE:'.format(self.vid, state_time))
            #log.info((planner_state.vehicle_state.get_S(), planner_state.vehicle_state.get_D()))
            
            #Maneuver Tick
            if mconfig and planner_state.lane_config:
                #replan maneuver
                traj, cand = plan_maneuver( mconfig.mkey,
                                            mconfig,
                                            np.concatenate([
                                                planner_state.vehicle_state.get_S(),
                                                planner_state.vehicle_state.get_D()]),
                                            planner_state.lane_config,
                                            planner_state.traffic_vehicles)
                if traj is None:
                    log.warn("plan_maneuver return invalid trajectory.")
                else:
                    self.write_motion_plan(mplan_sharr, traj, cand, state_time, ref_path_changed, mconfig.mkey == Maneuver.M_REVERSE)
            else:
                traj, cand = None, None

            #Write down debug info (for Dahsboard and Log)
            # change ref path format for pickling (maybe always keep it like this?)
            debug_ref_path = [(pt.x, pt.y) for pt in self.reference_path]
            debug_shdata[int(self.vid)] = (
                planner_state.vehicle_state, snapshot_tree, traj, cand, planner_state.traffic_vehicles,
                planner_state.lane_config, debug_ref_path)

        log.info('PLANNER PROCESS END')
        # shm_vs.close()
        # shm_vp.close()

    def get_planner_state(self, planner_tick:TickSync, vehicle_state:VehicleState, traffic_vehicles:List):
        """ Transforms vehicle_state and all traffic vehicles to the current frenet frame, and generates other
            frame-dependent planning data like current lane config and goal.
        """
        # update vehicle frenet state since refernce path may have changed
        s_vector, d_vector = sim_to_frenet_frame(self.reference_path, vehicle_state.get_X(), vehicle_state.get_Y())
        vehicle_state.set_S(s_vector)
        vehicle_state.set_D(d_vector)

        # update lane config based on current (possibly outdated) reference frame
        lane_config = self.read_map(vehicle_state, self.reference_path)
        if not lane_config:
            # No map data for current position
            log.warn("no lane config")
            return None

        # transform other vehicles to frenet frame based on this vehicle
        for vid, vehicle in traffic_vehicles.items():
            s_vector, d_vector = sim_to_frenet_frame(
                self.reference_path, vehicle.vehicle_state.get_X(), vehicle.vehicle_state.get_Y())
            vehicle.vehicle_state.set_S(s_vector)
            vehicle.vehicle_state.set_D(d_vector)

        # tranform goal to frenet
        goal_point_frenet = sim_to_frenet_position(self.reference_path, *self.sim_config.goal_points[self.vid])

        return PlannerState(
            sim_time=planner_tick.sim_time,
            vehicle_state=vehicle_state,
            lane_config=lane_config,
            goal_point_frenet=goal_point_frenet,
            traffic_vehicles=traffic_vehicles,
            pedestrians=None,
            obstacles=None
        )

    def read_map(self, vehicle_state, reference_path):
        """ Builds a lane config centered around the closest lanelet to vehicle_state lying
            on the reference_path.
        """
        cur_ll = self.laneletmap.get_occupying_lanelet_in_reference_path(
            reference_path, self.sim_config.lanelet_routes[self.vid], vehicle_state.x, vehicle_state.y)
        if not cur_ll:
            # as last resort, search the whole map
            cur_ll = self.laneletmap.get_occupying_lanelet(vehicle_state.x, vehicle_state.y)
            if not cur_ll:
                return None

        middle_lane_width = LaneletMap.get_lane_width(cur_ll, vehicle_state.x, vehicle_state.y)
        # LaneConfig(id, velocity, leftbound, rightbound).
        # /2 to center it on its centerline
        middle_lane_config = LaneConfig(0, 30, middle_lane_width / 2, middle_lane_width / -2)

        if self.laneletmap.get_left(cur_ll):
            upper_lane_width = LaneletMap.get_lane_width(self.laneletmap.get_left(cur_ll), vehicle_state.x, vehicle_state.y)
            upper_lane_config = LaneConfig(1, 30, middle_lane_config.left_bound + upper_lane_width, middle_lane_config.left_bound)
            middle_lane_config.set_left_lane(upper_lane_config)

        if self.laneletmap.get_right(cur_ll):
            lower_lane_width = LaneletMap.get_lane_width(self.laneletmap.get_right(cur_ll), vehicle_state.x, vehicle_state.y)
            lower_lane_config = LaneConfig(-1, 30, middle_lane_config.right_bound, middle_lane_config.right_bound - lower_lane_width)
            middle_lane_config.set_right_lane(lower_lane_config)

        return middle_lane_config

    def read_traffic_state(self, traffic_state_sharr):
        from sv.SV import Vehicle

        nv = self.nvehicles
        r = nv + 1
        c = int(len(traffic_state_sharr) / r)

        traffic_state_sharr.acquire() #<=========LOCK
        #header
        header_vector = traffic_state_sharr[0:3]
        #vehicles
        vehicles = {}
        my_vehicle_state = VehicleState()
        for ri in range(1,r):
            i = ri * c  #first index for row
            vid = traffic_state_sharr[i]
            remote = traffic_state_sharr[i+1]
            # state vector contains the vehicle's sim state and frenet state in its OWN ref path
            state_vector = traffic_state_sharr[i+2:i+c]
            if (vid == self.vid):
                my_vehicle_state.set_state_vector(state_vector)
            else:
                vehicle = Vehicle(vid)
                vehicle.vehicle_state.set_state_vector(state_vector)
                vehicles[vid] = vehicle
        traffic_state_sharr.release() #<=========RELEASE
        return header_vector, my_vehicle_state, vehicles

    def write_motion_plan(self, mplan_sharr, traj, cand, state_time, new_frenet_frame, reversing):
        if not traj:
            return
        plan = MotionPlan()
        plan.set_trajectory(traj[0], traj[1], traj[2])
        plan.t_start = state_time
        plan.new_frenet_frame = new_frenet_frame
        plan.reversing = reversing

        #write motion plan
        mplan_sharr.acquire() #<=========LOCK
        mplan_sharr[:] = plan.get_plan_vector()
        #print('Writting Sh Data VP')
        #print(mplan_sharr)
        mplan_sharr.release() #<=========RELEASE

    def __del__(self):
        if self._process:
            self._process.join()
