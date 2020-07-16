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
from TickSync import TickSync
from sv.VehicleState import *
from sv.ManeuverConfig import *
from sv.ManeuverModels import *
from util.Transformations import sim_to_frenet_frame, sim_to_frenet_position
from mapping.LaneletMap import LaneletMap
from sv.BTreeModel import *

from typing import Tuple, Dict, List

@dataclass
class PlannerState:
    sim_time:float
    vehicle_state:VehicleState
    lane_config:LaneConfig
    traffic_vehicles:Dict
    pedestrians:List
    obstacles:List
    goal_point:Tuple = None
    goal_point_frenet:Tuple = None


class SVPlanner(object):
    def __init__(self, vid, btree_root, nvehicles, laneletmap, sim_config, traffic_state_sharr, debug_shdata):
        #MainProcess space:
        self._process = None
        self._traffic_state_sharr = traffic_state_sharr
        self._debug_shdata = debug_shdata
        self._mplan_sharr = None
        self.motion_plan = None
        #Shared space
        self.vid = vid
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
        self._mplan_sharr = Array('f', c )
        #Process based
        self._process = Process(target=self.run_planner_process, args=(self._traffic_state_sharr, self._mplan_sharr, self._debug_shdata ), daemon = True)  
        self._process.start()
    
    def stop(self):
        if self._process:
            print("Terminate Planner Process - vehicle {}".format(self.vid))
            self._process.terminate()
        

    def get_plan(self):
        plan = MotionPlan()
        self._mplan_sharr.acquire() #<=========LOCK
        plan.set_plan_vector(self._mplan_sharr[:])
        self._mplan_sharr.release() #<=========RELEASE
        if (plan.t == 0): #if not valid
            return None
        if (self.motion_plan): #if same as current
            if (np.array_equal(self.motion_plan.get_plan_vector(),plan.get_plan_vector())): 
                return None
        #Valid and New:  
        self.motion_plan = plan
        return self.motion_plan
    
    #==SUB PROCESS=============================================

    def run_planner_process(self, traffic_state_sharr, mplan_sharr, debug_shdata ):
        print('PLANNER PROCESS START for Vehicle {}'.format(self.vid))
        
        sync_planner = TickSync(rate=PLANNER_RATE, realtime = True, block=True, verbose=False, label="PP")

        self.btree_model = BTreeModel(self.vid, self.btree_root)
        
        while sync_planner.tick():
            header, vehicle_state, traffic_vehicles = self.read_traffic_state(traffic_state_sharr)
            state_time = header[2]

            if self.reference_path is None:
                self.reference_path = self.laneletmap.get_global_path_for_route(self.sim_config.lanelet_routes[self.vid], vehicle_state.x, vehicle_state.y)
            
            # update lane config based on current (possibly outdated) reference frame
            lane_config = self.read_map(vehicle_state, self.reference_path)
            if not lane_config:
                # No map data for current position
                print("no lane config")
                continue

            # transform other vehicles to frenet frame based on this vehicle
            for vid, vehicle in traffic_vehicles.items():
                s_vector, d_vector = sim_to_frenet_frame(self.reference_path, vehicle.vehicle_state.get_X(), vehicle.vehicle_state.get_Y())
                vehicle.vehicle_state.set_S(s_vector)
                vehicle.vehicle_state.set_D(d_vector)

            #BTree Tick - using frenet state and lane config based on old ref path
            planner_state = PlannerState(
                sim_time=sync_planner.sim_time,
                vehicle_state=vehicle_state,
                lane_config=lane_config,
                goal_point_frenet=sim_to_frenet_position(self.reference_path, *self.sim_config.goal_points[self.vid]),
                traffic_vehicles=traffic_vehicles,
                pedestrians=None,
                obstacles=None
            )
            mconfig, ref_path_changed = self.btree_model.tick(planner_state)
            # when ref path changes, must recalculate the path, lane config and relative state of other vehicles
            if ref_path_changed:
                print("PATH CHANGED")
                self.reference_path = self.laneletmap.get_global_path_for_route(self.sim_config.lanelet_routes[self.vid], vehicle_state.x, vehicle_state.y)
                lane_config = self.read_map(vehicle_state, self.reference_path)
                # transform other vehicles to frenet frame based on this vehicle
                for vid, vehicle in traffic_vehicles.items():
                    s_vector, d_vector = sim_to_frenet_frame(self.reference_path, vehicle.vehicle_state.get_X(), vehicle.vehicle_state.get_Y())
                    vehicle.vehicle_state.set_S(s_vector)
                    vehicle.vehicle_state.set_D(d_vector)
            
            # since global path can change in this frame, frenet state in vehicle_state may be invalid
            new_s_vector, new_d_vector = sim_to_frenet_frame(self.reference_path, vehicle_state.get_X(), vehicle_state.get_Y())
            #print('Plan {} at time {} and FRENET STATE:'.format(self.vid, state_time))
            #print((new_s_vector[0], new_d_vector[0]))
            
            #Maneuver Tick
            if mconfig and lane_config:
                #replan maneuver
                traj, cand = plan_maneuver( mconfig.mkey,
                                            mconfig,
                                            np.concatenate([new_s_vector, new_d_vector]),
                                            lane_config,
                                            traffic_vehicles)
                self.write_motion_plan(mplan_sharr, traj, cand, state_time, ref_path_changed)
            
            #Write down debug info (for Dahsboard and Log)
            debug_shdata[self.vid] = (vehicle_state, traj, cand, traffic_vehicles, lane_config) 


        print('PLANNER PROCESS END')
        # shm_vs.close()
        # shm_vp.close()
    
    def read_map(self, vehicle_state, reference_path):
        """ Builds a lane config centered around the closest lanelet to vehicle_state lying
            on the reference_path.
        """
        cur_ll = self.laneletmap.get_occupying_lanelet_in_reference_path(reference_path, self.sim_config.lanelet_routes[self.vid], vehicle_state.x, vehicle_state.y)
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
        r = nv+1
        c = VehicleState.VECTORSIZE + VehicleState.FRENET_VECTOR_SIZE + 1

        traffic_state_sharr.acquire() #<=========LOCK
        #header
        header_vector = traffic_state_sharr[0:3]
        #vehicles
        vehicles = {}
        my_vehicle_state = VehicleState()
        for ri in range(1,r):
            i = ri * c  #first index for row
            vid = traffic_state_sharr[i]
            # state vector contains the vehicle's sim state and frenet state in its OWN ref path
            state_vector = traffic_state_sharr[i+1:i+c]
            if (vid == self.vid):
                my_vehicle_state.set_state_vector(state_vector)
            else:
                vehicle = Vehicle(vid)
                vehicle.vehicle_state.set_state_vector(state_vector)
                vehicles[vid] = vehicle
        traffic_state_sharr.release() #<=========RELEASE
        return header_vector, my_vehicle_state, vehicles

    def write_motion_plan(self, mplan_sharr, traj, cand, state_time, new_frenet_frame):
        if not traj:
            return
        plan = MotionPlan()
        plan.set_trajectory(traj[0],traj[1],traj[2])
        plan.t_start = state_time
        plan.new_frenet_frame = new_frenet_frame

        #write motion plan
        mplan_sharr.acquire() #<=========LOCK
        mplan_sharr[:] = plan.get_plan_vector()
        #print('Writting Sh Data VP')
        #print(mplan_sharr)
        mplan_sharr.release() #<=========RELEASE


    def __del__(self):
        if self._process:
            self._process.join()
