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
from Constants import *
from Utils import *
from ManeuverModels import *
from VehicleState import *
from TickSync import TickSync

#BTree #todo: pytrees
BT_PARKED = 0 #default, car is stopped
BT_DRIVE = 1  #follow a route with normal driving. Can switch to follow, or stop
BT_STOP = 2
BT_VELKEEP = 3
BT_FOLLOW = 4 #follow a specific target
BT_CUTIN = 5  #reach and cut in a specific target

class SVPlanner(object):
    def __init__(self, vid, nvehicles, laneletmap , traffic_state_sharr): #lock_vs, shm_vs, lock_vp, shm_vp):
        #MainProcess space:
        self._process = None
        self._traffic_state_sharr = traffic_state_sharr
        self._mplan_sharr = None
        self.motion_plan = None
        #Shared space
        self.vid = vid
        self.nvehicles = nvehicles
        self.laneletmap = laneletmap
        self.lookahead_dist = 10
        self.PLANNER_RATE = 5
    
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
        
        sync_planner = TickSync(rate=self.PLANNER_RATE, realtime = True, block=True, verbose=True, label="PP")
        traffic_state = None
        
        while sync_planner.tick():
            vehicle_state, header = self.read_traffic_state(traffic_state_sharr)
            state_time = header[2]
            
            #TODO: convert from Sim Frame to FrenetFrame using LaneConfig
            vehicle_frenet_state = np.concatenate([ vehicle_state.get_X(), vehicle_state.get_Y()])
            print('Plan at time {} and FRENET STATE:'.format(state_time))
            print(vehicle_frenet_state)
            
            #Access lane config based on vehicle_state
            lane_config = self.read_map(vehicle_frenet_state)

            #BTree Tick
            mkey, mconfig = self.behavior_tick(vehicle_frenet_state)
            #if man_key == M_LANECHANGE:
            #    lane_config = LaneConfig(1,30,4,0) #standard

            #Maneuver Tick
            if (mkey):
                #replan maneuver
                traj, cand = plan_maneuver( mkey, 
                                            mconfig, 
                                            vehicle_frenet_state, 
                                            lane_config, 
                                            traffic_state)
                man_key = None
                self.write_motion_plan(mplan_sharr, traj, cand, state_time)

        print('PLANNER PROCESS END')
        shm_vs.close()
        shm_vp.close()
    
    def read_map(self, frenet_state):
        #TODO: retrieve lane state from map
        #hardcoding now
        lower_lane_config = LaneConfig(1,30,4,0)
        upper_lane_config = LaneConfig(2,30,8,4)
        lower_lane_config.set_left_lane(upper_lane_config)
        
        d_pos = frenet_state[3]
        if ( 0 <= d_pos < 4):
            return lower_lane_config
        if ( 4 <= d_pos <= 8):
            return upper_lane_config

    
    def behavior_tick(self,frenet_state):
        #TODO: retrieve decision from BTree
        #hardcoding now
        mkey=M_VELKEEP
        mconfig = MVelKeepConfig()
        
        s_pos = frenet_state[0]
        d_pos = frenet_state[3]
        if (self.vid ==1): #lane changing vehicle
            if 0 <= s_pos < 20:
                mkey=M_VELKEEP
                mconfig = MVelKeepConfig()
            if 20 <= s_pos < 70:
                mkey=M_LANESWERVE
                mconfig = MLaneSwerveConfig(target_lid=2)
            #if 40 <= s_pos <80:
                #print('200 300')
            #    man_key=M_LANECHANGE
            #    man_config = MLaneChangeConfig((MIN_VELOCITY + 20, MAX_VELOCITY+20), (VK_MIN_TIME,VK_MAX_TIME))
            #if 80 <= s_pos:
            #    man_config = MVelKeepingConfig((MIN_VELOCITY, MAX_VELOCITY), (VK_MIN_TIME,VK_MAX_TIME)) 
            #man_key=M_LANECHANGE
            #man_config = MLaneChangeConfig((MIN_VELOCITY + 20, MAX_VELOCITY+20), (VK_MIN_TIME,VK_MAX_TIME))
        return mkey, mconfig, 

    def read_traffic_state(self, traffic_state_sharr):
        nv = self.nvehicles
        r = nv+1
        c = VehicleState.VECTORSIZE + 1
        traffic_state_sharr.acquire() #<=========LOCK
        #header
        header_vector = traffic_state_sharr[0:3]
        #vehicles
        vehicle_state = VehicleState()
        for ri in range(1,r):
            i = ri * c  #first index for row
            print(traffic_state_sharr[i])
            if (traffic_state_sharr[i] == self.vid):
                sv = traffic_state_sharr[i+1:i+c]
                vehicle_state.set_state_vector(sv)
        traffic_state_sharr.release() #<=========RELEASE
        return vehicle_state, header_vector

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