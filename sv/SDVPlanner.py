#!/usr/bin/env python
#rqueiroz@uwaterloo.ca
#d43sharm@uwaterloo.ca
# --------------------------------------------
# GEOSCENARIO SIMULATION VEHICLE PLANNER
# --------------------------------------------

import datetime
import numpy as np
from multiprocessing import shared_memory, Process, Lock, Array, Manager
from typing import Dict, List
import glog as log
from copy import copy
from TickSync import TickSync
from mapping.LaneletMap import LaneletMap
from sv.ManeuverConfig import *
from sv.ManeuverModels import plan_maneuver
from sv.btree.BehaviorModels import BehaviorModels
from sv.SDVPlannerState import PlannerState, TrafficLightState
from sv.SDVRoute import SDVRoute
from util.Transformations import sim_to_frenet_frame, sim_to_frenet_position, frenet_to_sim_frame, frenet_to_sim_position, OutsideRefPathException
import lanelet2.core
from mapping.LaneletMap import *
from Actor import *
from sv.FrenetTrajectory import *
from SimTraffic import *

import time

class SVPlanner(object):
    def __init__(self, sdv, sim_traffic, btree_locations):
        #MainProcess space:
        self._process = None
        self.traffic_state_sharr = sim_traffic.traffic_state_sharr
        self._traffic_light_sharr = sim_traffic.traffic_light_sharr
        self._debug_shdata = sim_traffic.debug_shdata
        self._mplan_sharr = None

        #Shared space
        self.vid = int(sdv.id)
        self.laneletmap:LaneletMap = sim_traffic.lanelet_map
        self.sim_config = sim_traffic.sim_config
        self.sim_traffic:SimTraffic = sim_traffic

        #Subprocess space
        self.root_btree_name = sdv.root_btree_name
        self.btree_reconfig = sdv.btree_reconfig
        self.behavior_model = None
        self.mconfig = None
        self.last_plan = None
        self.btree_locations = btree_locations
        self.btype = sdv.btype
        self.sdv_route = None

    def start(self):
        #Create shared arrray for Motion Plan
        c = MotionPlan().get_vector_length()
        self._mplan_sharr = Array('f', c)
        #Process based
        self._process = Process(target=self.run_planner_process, args=(
            self.traffic_state_sharr, 
            self._mplan_sharr, 
            self._debug_shdata), daemon=True)
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
        plan.set_plan_vector(copy(self._mplan_sharr[:]))
        self._mplan_sharr.release() #<=========RELEASE
        if (plan.trajectory.T == 0):
            # Empty plan
            return None
        elif (self.last_plan is not None) and (plan.tick_count == self.last_plan.tick_count):
            # Same plan
            return None
        # New plan
        self.last_plan = plan
        return plan
        

    #==SUB PROCESS=============================================

    def run_planner_process(self, traffic_state_sharr, mplan_sharr, debug_shdata):
        log.info('PLANNER PROCESS START for Vehicle {}'.format(self.vid))

        sync_planner = TickSync(rate=PLANNER_RATE, realtime=True, block=True, verbose=False, label="PP")
        sync_planning_task = TickSync(rate=PLANNER_RATE, realtime=True, block=True, verbose=False, label="PP")

        #Behavior Layer
        #Note: If an alternative behavior module is to be used, it must be replaced here.
        self.behavior_model = BehaviorModels(self.vid, self.root_btree_name, self.btree_reconfig, self.btree_locations, self.btype)
        
        # target time for planning task. Can be fixed or variable up to max planner tick time
        task_label = "V{} plan".format(self.vid)
        if USE_FIXED_PLANNING_TIME:
            sync_planner.set_task(task_label,PLANNING_TIME)
        else:
            sync_planner.set_task(task_label,PLANNING_TIME,1/PLANNER_RATE)

        while sync_planner.tick():
            sync_planner.start_task()

            # Get sim state from main process
            # All objects are copies and can be changed
            header, traffic_vehicles, traffic_pedestrians,traffic_light_states, static_objects = self.sim_traffic.read_traffic_state(traffic_state_sharr, True)
            state_time = header[2]
            tick_count = header[0]
            if self.vid in traffic_vehicles:
                vehicle_state = traffic_vehicles.pop(self.vid, None).state #removes self state
            else:
                #vehicle state not available. Vehicle can be inactive.
                continue
            
            if self.sdv_route is None:
                self.sdv_route = SDVRoute(
                    self.sim_config.lanelet_routes[self.vid], self.laneletmap, vehicle_state.x, vehicle_state.y
                )

            # Get traffic, lane config and regulatory elements in current frenet frame
            self.project_dynamic_objects(vehicle_state, traffic_vehicles, traffic_pedestrians, state_time, sync_planner.get_task_time())
            planner_state = self.get_planner_state(sync_planner, vehicle_state, traffic_vehicles, traffic_pedestrians, traffic_light_states, static_objects)
            if not planner_state:
                log.warn("Invalid planner state, skipping planning step...")
                continue
            
            #BTree Tick - using frenet state and lane config based on old ref path
            mconfig, ref_path_changed, snapshot_tree = self.behavior_model.tick(planner_state)
            
            # when ref path changes, must recalculate the path, lane config and relative state of other vehicles
            if ref_path_changed:
                log.info("PATH CHANGED")

                self.sdv_route.update_global_path(vehicle_state.x, vehicle_state.y)

                # Regenerate planner state and tick btree again. Discard whether ref path changed again.
                planner_state = self.get_planner_state(sync_planner, vehicle_state, traffic_vehicles, traffic_pedestrians, traffic_light_states, static_objects)
                if not planner_state:
                    log.warn("Invalid planner state, skipping planning step...")
                    continue
                mconfig, _, snapshot_tree = self.behavior_model.tick(planner_state)
            
            # new maneuver
            if self.mconfig and self.mconfig.mkey != mconfig.mkey:
                log.info("VID {} started maneuver {}".format(self.vid, mconfig.mkey.name))
                # print sv state and deltas
                state_str = (
                    "VID {}:\n"
                    "   position    s={:.3f} sim=({:.3f},{:.3f})\n"
                    "   speed       {:.3f}\n"
                ).format(
                    self.vid,
                    vehicle_state.s,
                    vehicle_state.x, vehicle_state.y,
                    vehicle_state.s_vel
                )
                for vid, tvehicle in traffic_vehicles.items():
                    state_str += (
                        "VID {}:\n"
                        "   position    {:.3f}\n"
                        "   speed       {:.3f}\n"
                        "   delta dist  {:.3f}\n"
                        "   delta vel   {:.3f}\n"
                    ).format(
                        vid,
                        tvehicle.state.s,
                        tvehicle.state.s_vel,
                        vehicle_state.s - tvehicle.state.s - 2*VEHICLE_RADIUS,
                        vehicle_state.s_vel - tvehicle.state.s_vel
                    )
                #log.info(state_str)
            self.mconfig = mconfig
            #Maneuver Tick
            if mconfig and planner_state.lane_config:
                #replan maneuver
                #traj, cand, unf = plan_maneuver( mconfig.mkey,
                frenet_traj, cand = plan_maneuver(self.vid, 
                                            mconfig,
                                            planner_state.vehicle_state,
                                            planner_state.lane_config,
                                            planner_state.traffic_vehicles,
                                            planner_state.pedestrians,
                                            planner_state.static_objects)

                if EVALUATION_MODE and not self.last_plan:
                    sync_planner.end_task(False) #blocks if < target
                    task_delta_time = 0
                else:
                    sync_planner.end_task() #blocks if < target
                    task_delta_time = sync_planner.get_task_time()

                if frenet_traj is None:
                    log.warn("plan_maneuver return invalid trajectory.")
                    pass
                else:
                    plan = MotionPlan()
                    plan.trajectory = frenet_traj
                    plan.start_time = state_time + task_delta_time
                    plan.new_frenet_frame = ref_path_changed
                    plan.reversing = mconfig.mkey == Maneuver.M_REVERSE
                    plan.tick_count = tick_count
                    plan.ref_path_origin = self.sdv_route.get_reference_path_origin()
                    self.write_motion_plan(mplan_sharr, plan)
                    if plan.trajectory.T > 0.0: #only for non zero traj
                        self.last_plan = plan
            else:
                frenet_traj, cand = None, None
            
            #Debug info (for Dahsboard and Log)
            if self.sim_config.show_dashboard:
                # change ref path format for pickling (maybe always keep it like this?)
                debug_ref_path = [(pt.x, pt.y) for pt in self.sdv_route.get_reference_path()]
                debug_shdata[int(self.vid)] = (
                    planner_state,
                    snapshot_tree,
                    debug_ref_path,
                    (mplan_sharr[:6], mplan_sharr[6:12], mplan_sharr[12]),
                    [traj.array_format() for traj in cand if traj.feasible] if cand else None,
                    [traj.array_format() for traj in cand if not traj.feasible] if cand else None
                )
        log.info('PLANNER PROCESS END. Vehicle{}'.format(self.vid))
        
    def project_dynamic_objects(
            self,
            vehicle_state:VehicleState,
            traffic_vehicles:dict,
            traffic_pedestrians:dict,
            state_time,
            expected_planner_time):

        if self.last_plan and expected_planner_time > 0:
            sim_time_ahead = state_time + expected_planner_time
            delta_time = sim_time_ahead - self.last_plan.start_time
            if (delta_time > self.last_plan.trajectory.T): 
                delta_time = self.last_plan.trajectory.T
            new_state = self.last_plan.trajectory.get_state_at(delta_time)
            vehicle_state.set_S(new_state[:3])
            vehicle_state.set_D(new_state[3:])

            self.sdv_route.update_reference_path(self.last_plan.ref_path_origin)

            try:
                x_vector, y_vector = frenet_to_sim_frame(
                    self.sdv_route.get_reference_path(), vehicle_state.get_S(),
                    vehicle_state.get_D(), self.sdv_route.get_reference_path_s_start()
                )
            except OutsideRefPathException:
                return

            vehicle_state.x = x_vector[0]
            vehicle_state.x_vel = x_vector[1]
            vehicle_state.x_acc = x_vector[2]
            vehicle_state.y = y_vector[0]
            vehicle_state.y_vel = y_vector[1]
            vehicle_state.y_acc = y_vector[2]

            for vid, vehicle in traffic_vehicles.items():
                state = vehicle.future_state(expected_planner_time)
                vehicle.state.s = state[0]
                vehicle.state.s_vel = state[1]
                vehicle.state.s_acc = state[2]
                vehicle.state.d = state[3]
                vehicle.state.d_vel = state[4]
                vehicle.state.d_acc = state[5]

            for pid, pedestrian in traffic_pedestrians.items():
                state = pedestrian.future_state(expected_planner_time)
                pedestrian.state.s = state[0]
                pedestrian.state.s_vel = state[1]
                pedestrian.state.s_acc = state[2]
                pedestrian.state.d = state[3]
                pedestrian.state.d_vel = state[4]
                pedestrian.state.d_acc = state[5]

    def get_planner_state(
            self,
            planner_tick:TickSync,
            vehicle_state:VehicleState,
            traffic_vehicles:dict,
            traffic_pedestrians:dict,
            traffic_light_states:dict,
            static_objects:dict):
        """ Transforms vehicle_state and all traffic vehicles to the current frenet frame, and generates other
            frame-dependent planning data like current lane config and goal.
        """

        s_vector, d_vector = sim_to_frenet_frame(
            self.sdv_route.get_global_path(), vehicle_state.get_X(), vehicle_state.get_Y(), 0
        )
        vehicle_state.set_S(s_vector)
        vehicle_state.set_D(d_vector)

        # the next plan's ref_path_origin is vehicle_state.s
        self.sdv_route.update_reference_path(vehicle_state.s, plan_lane_change=False)
        vehicle_state.s = 0.0

        # update lane config based on current (possibly outdated) reference frame
        lane_config, reg_elems = self.read_map(vehicle_state, traffic_light_states)
        if not lane_config:
            # No map data for current position
            log.warn("no lane config")
            return None

        # transform other vehicles and pedestrians to frenet frame based on this vehicle
        for vid, vehicle in list(traffic_vehicles.items()):
            try:
                s_vector, d_vector = sim_to_frenet_frame(
                    self.sdv_route.get_reference_path(), vehicle.state.get_X(),
                    vehicle.state.get_Y(), self.sdv_route.get_reference_path_s_start()
                )
                vehicle.state.set_S(s_vector)
                vehicle.state.set_D(d_vector)
            except OutsideRefPathException:
                del traffic_vehicles[vid]
        
        for pid, pedestrian in list(traffic_pedestrians.items()):
            try:
                s_vector, d_vector = sim_to_frenet_frame(
                        self.sdv_route.get_reference_path(), pedestrian.state.get_X(),
                        pedestrian.state.get_Y(), self.sdv_route.get_reference_path_s_start()
                    )
                pedestrian.state.set_S(s_vector)
                pedestrian.state.set_D(d_vector)
            except OutsideRefPathException:
                del traffic_pedestrians[pid]
        
        for soid, so in list(static_objects.items()):
            try:
                so.s, so.d = sim_to_frenet_position(
                    self.sdv_route.get_reference_path(), so.x,
                    so.y, self.sdv_route.get_reference_path_s_start()
                )
            except OutsideRefPathException:
                del static_objects[soid]

        # transform goal to frenet
        try:
            goal_point_frenet = sim_to_frenet_position(
                self.sdv_route.get_reference_path(),
                *self.sim_config.goal_points[self.vid], self.sdv_route.get_reference_path_s_start()
            )
        except OutsideRefPathException:
            goal_point_frenet = None

        return PlannerState(
            sim_time=planner_tick.sim_time,
            vehicle_state=vehicle_state,
            lane_config=lane_config,
            goal_point_frenet=goal_point_frenet,
            traffic_vehicles=traffic_vehicles,
            regulatory_elements=reg_elems,
            pedestrians=traffic_pedestrians,
            static_objects=static_objects
        )

    def read_map(self, vehicle_state, traffic_light_states):
        """ Builds a lane config centered around the closest lanelet to vehicle_state lying
            on the reference_path.
        """
        cur_ll = self.laneletmap.get_occupying_lanelet_in_reference_path(
            self.sdv_route.get_reference_path(),
            self.sim_config.lanelet_routes[self.vid],vehicle_state.x, vehicle_state.y
        )
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

        # Get regulatory elements acting on this lanelet
        reg_elems = cur_ll.regulatoryElements
        reg_elem_states = []
        for re in reg_elems:
            if isinstance(re, lanelet2.core.TrafficLight):
                # lanelet2 traffic lights must have a corresponding state from the main process
                if re.id not in traffic_light_states:
                    continue

                stop_linestring = re.parameters['ref_line']

                try:
                    # choose the closest point on the stop line as the stop position
                    stop_pos = min(
                            sim_to_frenet_position(
                                self.sdv_route.get_reference_path(), stop_linestring[0][0].x,
                                stop_linestring[0][0].y, self.sdv_route.get_reference_path_s_start(),
                                max_dist_from_path=10.0
                            ),
                            sim_to_frenet_position(
                                self.sdv_route.get_reference_path(), stop_linestring[0][-1].x,
                                stop_linestring[0][-1].y, self.sdv_route.get_reference_path_s_start(),
                                max_dist_from_path=10.0
                            ),
                            key=lambda p: p[0]
                    )
                    reg_elem_states.append(TrafficLightState(color=traffic_light_states[re.id], stop_position=stop_pos))
                except OutsideRefPathException:
                    pass

        return middle_lane_config, reg_elem_states

    def write_motion_plan(self, mplan_sharr, plan:MotionPlan): 
        if not plan:
            return
        #write motion plan
        mplan_sharr.acquire() #<=========LOCK
        mplan_sharr[:] = plan.get_plan_vector()
        #print('Writting Sh Data VP')
        #print(mplan_sharr)
        mplan_sharr.release() #<=========RELEASE
        
