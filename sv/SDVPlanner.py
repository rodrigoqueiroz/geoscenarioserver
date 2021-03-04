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
# from sv.btree.BTreeModel import * # Deprecated
#from sv.btree.BTreeFactory import *
from sv.btree.BehaviorModels import BehaviorModels
from sv.SDVPlannerState import PlannerState, TrafficLightState
# import sv.SV
from util.Transformations import sim_to_frenet_frame, sim_to_frenet_position
import lanelet2.core
from mapping.LaneletMap import *
from Actor import *
from sv.FrenetTrajectory import *
from SimTraffic import *


class SVPlanner(object):
    def __init__(self, sdv, sim_traffic):
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
        # Reference path that the planner will use for all transformations and planning
        self.reference_path = None
        self.btree_root = sdv.btree_root
        self.btree_reconfig = sdv.btree_reconfig
        self.behavior_model = None
        self.mconfig = None

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
        #if empty
        if (plan.trajectory.T == 0): 
            return None
        #Valid:
        return plan
        

    #==SUB PROCESS=============================================

    def run_planner_process(self, traffic_state_sharr, mplan_sharr, debug_shdata):
        log.info('PLANNER PROCESS START for Vehicle {}'.format(self.vid))

        sync_planner = TickSync(rate=PLANNER_RATE, realtime=True, block=True, verbose=False, label="PP")

        #Behavior Layer
        #Note: If an alternative behavior module is to be used, it must be replaced here.
        self.behavior_model = BehaviorModels(self.vid, self.btree_root,  self.btree_reconfig)
        
        while sync_planner.tick():
            TickSync.clock_log("Planner: start")
            # Get sim state from main process
            # All objects are copies and can be changed
            header, traffic_vehicles, traffic_pedestrians,traffic_light_states, static_objects = self.sim_traffic.read_traffic_state(traffic_state_sharr, True)
            state_time = header[2]
            if self.vid in traffic_vehicles:
                vehicle_state = traffic_vehicles.pop(self.vid, None).state #removes self state
            else:
                #vehicle state not available. Vehicle can be inactive.
                continue
            TickSync.clock_log("Planner: read traffic")

            if self.reference_path is None:
                self.reference_path = self.laneletmap.get_global_path_for_route(
                    self.sim_config.lanelet_routes[self.vid], vehicle_state.x, vehicle_state.y)

            # Get traffic, lane config and regulatory elements in current frenet frame
            planner_state = self.get_planner_state(sync_planner, vehicle_state, traffic_vehicles, traffic_pedestrians, traffic_light_states,static_objects)
            if not planner_state:
                log.warn("Invalid planner state, skipping planning step...")
                continue
            TickSync.clock_log("Planner: traffic state")

            #BTree Tick - using frenet state and lane config based on old ref path
            mconfig, ref_path_changed, snapshot_tree = self.behavior_model.tick(planner_state)
            TickSync.clock_log("Planner: behavior")

            # when ref path changes, must recalculate the path, lane config and relative state of other vehicles
            if ref_path_changed:
                log.info("PATH CHANGED")

                self.reference_path = self.laneletmap.get_global_path_for_route(
                    self.sim_config.lanelet_routes[self.vid], vehicle_state.x, vehicle_state.y)

                # Regenerate planner state and tick btree again. Discard whether ref path changed again.
                planner_state = self.get_planner_state(sync_planner, vehicle_state, traffic_vehicles, traffic_pedestrians, traffic_light_states, static_objects)
                if not planner_state:
                    log.warn("Invalid planner state, skipping planning step...")
                    continue
                mconfig, _, snapshot_tree = self.behavior_model.tick(planner_state)
            TickSync.clock_log("Planner: new ref path")

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
                log.info(state_str)
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
                if frenet_traj is None:
                    log.warn("plan_maneuver return invalid trajectory.")
                    pass
                else:
                    plan = MotionPlan()
                    plan.trajectory = frenet_traj
                    plan.start_time = state_time
                    plan.new_frenet_frame = ref_path_changed
                    plan.reversing = mconfig.mkey == Maneuver.M_REVERSE
                    self.write_motion_plan(mplan_sharr, plan)
                    if plan.trajectory.T > 0.0: #only for non zero traj
                        #print("planner {} wrote".format(self.vid))
                        #print(plan)
                        self.last_plan = plan
            else:
                traj, cand = None, None
            TickSync.clock_log("Planner: maneuver")

            if self.sim_config.show_dashboard:
                #Write down debug info (for Dahsboard and Log)
                # change ref path format for pickling (maybe always keep it like this?)
                debug_ref_path = [(pt.x, pt.y) for pt in self.reference_path]
                debug_shdata[int(self.vid)] = (
                    planner_state,
                    snapshot_tree,
                    debug_ref_path,
                    (mplan_sharr[:6], mplan_sharr[6:12], mplan_sharr[12]),
                    [traj.array_format() for traj in cand if traj.feasible] if cand else None,
                    [traj.array_format() for traj in cand if not traj.feasible] if cand else None
                )

            TickSync.clock_log("Planner: debug")
            #TickSync.print_clock_log()
            

        log.info('PLANNER PROCESS END')
        

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
        # update vehicle frenet state since refernce path may have changed
        s_vector, d_vector = sim_to_frenet_frame(self.reference_path, vehicle_state.get_X(), vehicle_state.get_Y())
        vehicle_state.set_S(s_vector)
        vehicle_state.set_D(d_vector)

        # update lane config based on current (possibly outdated) reference frame
        lane_config, reg_elems = self.read_map(vehicle_state, self.reference_path, traffic_light_states)
        if not lane_config:
            # No map data for current position
            log.warn("no lane config")
            return None

        # transform other vehicles and pedestrians to frenet frame based on this vehicle
        for vid, vehicle in traffic_vehicles.items():
            s_vector, d_vector = sim_to_frenet_frame(
                self.reference_path, vehicle.state.get_X(), vehicle.state.get_Y())
            vehicle.state.set_S(s_vector)
            vehicle.state.set_D(d_vector)
        
        for pid, pedestrian in traffic_pedestrians.items():
            s_vector, d_vector = sim_to_frenet_frame(
                self.reference_path, pedestrian.state.get_X(), pedestrian.state.get_Y())
            pedestrian.state.set_S(s_vector)
            pedestrian.state.set_D(d_vector)
        
        for soid, so in static_objects.items():
            so.s, so.d = sim_to_frenet_position(self.reference_path, so.x, so.y)

        # transform goal to frenet
        goal_point_frenet = sim_to_frenet_position(self.reference_path, *self.sim_config.goal_points[self.vid])

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

    def read_map(self, vehicle_state, reference_path, traffic_light_states):
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

        # Get regulatory elements acting on this lanelet
        reg_elems = cur_ll.regulatoryElements
        reg_elem_states = []
        for re in reg_elems:
            if isinstance(re, lanelet2.core.TrafficLight):
                # lanelet2 traffic lights must have a corresponding state from the main process
                if re.id not in traffic_light_states:
                    continue

                stop_linestring = re.parameters['ref_line']
                # choose the closest point on the stop line as the stop position
                stop_pos = min(
                    sim_to_frenet_position(self.reference_path, stop_linestring[0][0].x, stop_linestring[0][0].y),
                    sim_to_frenet_position(self.reference_path, stop_linestring[0][-1].x, stop_linestring[0][-1].y),
                    key=lambda p: p[0])
                reg_elem_states.append(TrafficLightState(color=traffic_light_states[re.id], stop_position=stop_pos))

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
        
