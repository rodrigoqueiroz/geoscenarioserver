#!/usr/bin/env python
#rqueiroz@gsd.uwaterloo.ca
# --------------------------------------------
# GEOSCENARIO SIMULATION VEHICLE PLANNER
# --------------------------------------------

import numpy as np
import random
import datetime
import time
from dataclasses import dataclass
from multiprocessing import shared_memory, Process, Lock, Array
from TickSync import TickSync
from sv.VehicleState import *
from sv.ManeuverConfig import *
from sv.ManeuverModels import *

from Mapping.LaneletMap import LaneletMap

#BTree #todo: pytrees
BT_PARKED = 0 #default, car is stopped
BT_DRIVE = 1  #follow a route with normal driving. Can switch to follow, or stop
BT_STOP = 2
BT_VELKEEP = 3
BT_FOLLOW = 4 #follow a specific target
BT_CUTIN = 5  #reach and cut in a specific target

class SVPlanner(object):
    def __init__(self, vid, nvehicles, laneletmap, sim_config, traffic_state_sharr): #lock_vs, shm_vs, lock_vp, shm_vp):
        #MainProcess space:
        self._process = None
        self._traffic_state_sharr = traffic_state_sharr
        self._mplan_sharr = None
        self.motion_plan = None
        #Shared space
        self.vid = vid
        self.nvehicles = nvehicles
        self.laneletmap = laneletmap
        self.sim_config = sim_config
        self.lookahead_dist = 10
        self.PLANNER_RATE = 5

        self.mconfig = None
        self.lane_config = None

    def start(self):
        #Create Shared arrray for Plan
        c = MotionPlan.VECTORSIZE
        self._mplan_sharr = Array('f', c )
        #Process based
        self._process = Process(target=self.run_planner_process, args=(self._traffic_state_sharr, self._mplan_sharr ), daemon = True)  
        self._process.start()
    
    def stop(self):
       pass 
       #print("Planner Process Join")
        #if (self._process):
            #self._process.join()

    
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

    def run_planner_process(self, traffic_state_sharr, mplan_sharr ):
        print('PLANNER PROCESS START for Vehicle {}'.format(self.vid))
        
        sync_planner = TickSync(rate=self.PLANNER_RATE, realtime = True, block=True, verbose=False, label="PP")
        
        while sync_planner.tick():
            header, vehicle_state, traffic_vehicles = self.read_traffic_state(traffic_state_sharr)
            state_time = header[2]

            #BTree Tick - using frenet state based on old ref path
            new_mconfig = self.behavior_tick(vehicle_state, sync_planner.sim_time)
            if self.mconfig is None or (new_mconfig):
                print("Got new maneuver: {}".format(new_mconfig.mkey))
                self.mconfig = new_mconfig
                # new maneuver: rebuild map
                self.lane_config = self.read_map(vehicle_state)
            
            # since global path can change in this frame, frenet state in vehicle_state may be invalid
            ref_path = self.laneletmap.get_global_path_for_route(self.sim_config.lanelet_routes[self.vid], vehicle_state.x, vehicle_state.y)
            s_vector, d_vector = self.laneletmap.sim_to_frenet_frame(ref_path, vehicle_state.get_X(), vehicle_state.get_Y())
            vehicle_state.set_S(s_vector)
            vehicle_state.set_D(d_vector)
            #print('Plan {} at time {} and FRENET STATE:'.format(self.vid, state_time))
            #print((vehicle_state.s, vehicle_state.d))
            
            # transform other vehicles to frenet frame based on this vehicle
            for vid, vehicle in traffic_vehicles.items():
                s_vector, d_vector = self.laneletmap.sim_to_frenet_frame(ref_path, vehicle.vehicle_state.get_X(), vehicle.vehicle_state.get_Y())
                vehicle.vehicle_state.set_S(s_vector)
                vehicle.vehicle_state.set_D(d_vector)

            #Maneuver Tick
            if (self.mconfig and self.lane_config):
                #replan maneuver
                traj, cand = plan_maneuver( self.mconfig.mkey, 
                                            self.mconfig, 
                                            np.concatenate([vehicle_state.get_S(), vehicle_state.get_D()]), 
                                            self.lane_config, 
                                            traffic_vehicles)
                self.write_motion_plan(mplan_sharr, traj, cand, state_time)

        print('PLANNER PROCESS END')
        shm_vs.close()
        shm_vp.close()
    
    def read_map(self, vehicle_state):
        # build lane configs centered around the current lanelet
        cur_ll, _ = self.laneletmap.get_occupying_lanelet_by_position(self.sim_config.lanelet_routes[self.vid], vehicle_state.x, vehicle_state.y)
        middle_lane_width = LaneletMap.get_lane_width(cur_ll, vehicle_state.x, vehicle_state.y)
        # LaneConfig(id, velocity, leftbound, rightbound). /2 to center it on its centerline
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

    
    def behavior_tick(self,vehicle_state, sim_time):
        #TODO: retrieve decision from BTree
        #hardcoding now
        #Standard
        mconfig = None

        if self.mconfig is None:
            mconfig = MVelKeepConfig()
        else:
            # Hardcoded lane change scenario - note s changes when ref path changes
            if 6 < sim_time and self.mconfig.mkey != M_VELKEEP:
                mconfig = MVelKeepConfig()
            if 3 < sim_time < 6:
                if self.mconfig.mkey != M_LANESWERVE:
                    mconfig = MLaneSwerveConfig(-1)
                else:
                    # use OLD frenet frame to check current lane position. if in target lane switch to velkeep
                    cur_lane_config = self.lane_config.get_current_lane(vehicle_state.d)
                    if cur_lane_config.id == self.mconfig.target_lid:
                        mconfig = MLaneSwerveConfig(0)
        
            # hardcoded follow scenario
            # if self.vid == 1:
            #     mconfig = MFollowConfig(2)
        return mconfig

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

    def write_motion_plan(self, mplan_sharr, traj, cand, state_time):
        if not traj:
            return
        plan = MotionPlan()
        plan.set_trajectory(traj[0],traj[1],traj[2])
        plan.t_start = state_time
        #write motion plan
        mplan_sharr.acquire() #<=========LOCK
        mplan_sharr[:] = plan.get_plan_vector()
        #print('Writting Sh Data VP')
        #print(mplan_sharr)
        mplan_sharr.release() #<=========RELEASE


    def __del__(self):
        if self._process:
            self._process.join()
