#!/usr/bin/env python
#rqueiroz@uwaterloo.ca
#d43sharm@uwaterloo.ca
# --------------------------------------------
# SIMULATED VEHICLES
# --------------------------------------------

from os import stat_result
from matplotlib import pyplot as plt
import math
import sys
import glog as log
import numpy as np
from TickSync import TickSync
from SimConfig import *
from util.Transformations import frenet_to_sim_frame, sim_to_frenet_frame, OutsideRefPathException
from util.Utils import *
from sv.SDVPlanner import *
from Actor import *
from mapping.LaneletMap import LaneletMap
from shm.SimSharedMemory import *
from util.Utils import kalman

import datetime

# Vehicle base class for remote control or simulation.
class Vehicle(Actor):
    #vehicle types
    N_TYPE = 0      #neutral
    SDV_TYPE = 1    
    EV_TYPE = 2         
    TV_TYPE = 3

    def __init__(self, id, name='', start_state=[0.0,0.0,0.0, 0.0,0.0,0.0], frenet_state=[0.0,0.0,0.0, 0.0,0.0,0.0]):
        super().__init__(id,name, start_state,frenet_state, VehicleState())
        self.type = Vehicle.N_TYPE
        self.radius = VEHICLE_RADIUS

    def update_sim_state(self, new_state, delta_time):
        # NOTE: this may desync the sim and frenet vehicle state, so this should
        # only be done for external vehicles (which don't have a frenet state)
        if self.type is not Vehicle.EV_TYPE:
            log.warn("Cannot update sim state for gs vehicles directly.")
            
    def get_full_state_for_client(self):
        x = round((self.state.x * CLIENT_METER_UNIT))
        y = round((self.state.y * CLIENT_METER_UNIT))
        z = 0.0
        position = [x, y, z]
        velocity = [self.state.x_vel, self.state.y_vel]
        #remote = 1 if self.is_remote else 0
        return self.id, self.type, position, velocity, self.state.yaw, self.state.steer


class SDV(Vehicle):
    ''''
    Simulated Driver-Vehicle Model (dynamic behavior)
    '''
    def __init__(self, vid, name, btree_root, start_state,  lanelet_map:LaneletMap, lanelet_route, start_state_in_frenet=False):
        #Map
        self.lanelet_map = lanelet_map
        self.lanelet_route = lanelet_route # list of lanelet ids we want this vehicle to follow
        self.global_path = None
        if start_state_in_frenet:
            # assume frenet start_state is relative to the first lane of the route
            self.global_path = self.lanelet_map.get_global_path_for_route(self.lanelet_route)
            # compute sim state
            x_vector, y_vector = frenet_to_sim_frame(self.global_path, start_state[0:3], start_state[3:])
            Vehicle.__init__(self, vid, name, start_state=(x_vector+y_vector), frenet_state=start_state)
        else:
            self.global_path = self.lanelet_map.get_global_path_for_route(self.lanelet_route, x=start_state[0], y=start_state[3])
            # Compute frenet state corresponding to start_state
            s_vector, d_vector = sim_to_frenet_frame(self.global_path, start_state[0:3], start_state[3:])
            Vehicle.__init__(self, vid, name, start_state=start_state, frenet_state=(s_vector + d_vector))
        self.type = Vehicle.SDV_TYPE
        #Planning
        self.sv_planner = None
        #Behavior
        self.btree_root = btree_root
        self.btree_reconfig = ""
        self.motion_plan = None
        self.next_motion_plan = None
        
        #debug:
        self.jump_back_count = 0
        self.max_jump_back_dist = 0

    def start_planner(self):
        """For SDV models controlled by SVPlanner.
            If a planner is started, the vehicle can't be a remote.
        """
        self.sv_planner = SVPlanner(self, self.sim_traffic)
        self.sv_planner.start()

    def stop(self):
        if self.sv_planner:
            self.sv_planner.stop()

    def tick(self, tick_count, delta_time, sim_time):
        Vehicle.tick(self, tick_count, delta_time, sim_time)
        #Read planner
        if self.sv_planner:
            plan = self.sv_planner.get_plan()
            if plan:
                if plan.trajectory.T != 0:
                    self.set_new_motion_plan(plan, sim_time)
                    if plan.new_frenet_frame:
                        # NOTE: vehicle state being used here is from the previous frame (that planner should have gotten)
                        self.global_path = self.lanelet_map.get_global_path_for_route(self.lanelet_route, self.state.x, self.state.y)
                else:
                    self.set_new_motion_plan(plan, sim_time)
            #Compute new state
            self.compute_vehicle_state(delta_time, sim_time)

    def compute_vehicle_state(self, delta_time, sim_time):
        """
        Consume trajectory based on a given time and update pose
        Optimized with pre computed derivatives and equations
        """
        
        #Check if vehicle can switch to new plan
        if (self.next_motion_plan):
            time = sim_time - self.next_motion_plan.start_time
            #if ahead of current im time
            if time < 0:
                log.warning("Next v{} plan at {:2f}s is ahead of sim_time {:2f}s (diff{:2f}s) and will be delayed".format(
                    self.id,self.next_motion_plan.start_time,sim_time, time))
            else:
                self.motion_plan = self.next_motion_plan
                self.next_motion_plan = None
        
        #Follow current plan
        if (self.motion_plan):
            time = sim_time - self.motion_plan.start_time
            
            #exceed total traj time
            #If this happens and the vehicle is not stopped, the planner is stuck and can't generate new trajectories.
            #note: add an emergency break /fallback as a backup plan
            if (time > self.motion_plan.trajectory.T): 
                if self.state.s_vel > 0.1:
                    log.error("Vehicle {} finished last trajectory without stopping".format(self.id))
                return

            # new state
            self.motion_plan.trajectory.consumed_time = time
            new_state = self.motion_plan.trajectory.get_state_at(time) 

            #check consistency during transition from previous to new states
            # <1cm is within acceptable difference (rounding and error in frame conversion)
            s_delta = new_state[0] - self.state.s
            if s_delta < 0 and abs(s_delta) > 0.001 and not self.motion_plan.reversing:
                log.error("Vehicle {} moved backwards by {}m".format(self.id, s_delta))
                log.info("SimTime {} Delta {} Plan: StartTime{} TimeIn {}".format(
                        sim_time, delta_time, self.motion_plan.start_time,time))
                log.info(self.state.get_frenet_state_vector())
                log.info(new_state)
                #log.info(self.motion_plan.trajectory)
                self.jump_back_count += 1
                self.max_jump_back_dist = max(self.max_jump_back_dist, s_delta)

            # update frenet state
            self.state.set_S(new_state[:3])
            self.state.set_D(new_state[3:])
            
            # Compute sim state using the global path this vehicle is following
            # self.global_path = self.lanelet_map.get_global_path_for_route(self.state.x, self.state.y, self.lanelet_route)
            try:
                x_vector, y_vector = frenet_to_sim_frame(self.global_path, self.state.get_S(), self.state.get_D())
            except OutsideRefPathException as e:
                # assume we've reached the goal and exit?
                log.error("Outside of path. Vehicle reached goal?")
                return
                #raise e
            
            # update sim state
            self.state.x = x_vector[0]
            self.state.x_vel = x_vector[1]
            self.state.x_acc = x_vector[2]
            self.state.y = y_vector[0]
            self.state.y_vel = y_vector[1]
            self.state.y_acc = y_vector[2]
            heading = np.array([y_vector[1], x_vector[1]])
            if self.motion_plan.reversing:
                heading *= -1
            self.state.yaw = math.degrees(math.atan2(heading[1], heading[0]))

            #DEBUG:
            #Note: use this log to evaluate if the "jump back" issue returns
            #log.info("===VID: %d, Plan Tick: %d, Timestamp: %f, Delta Time: %f, Diff: %f, Start Time: %f, S Coef: (%f, %f, %f, %f, %f, %f)===" % (
            #    self.id, self.motion_plan.tick_count,
            #    datetime.datetime.now().timestamp(), delta_time,
            #    diff, self.motion_plan.start_time,
            #    self.motion_plan.trajectory.s_coef[0], self.motion_plan.trajectory.s_coef[1],
            #    self.motion_plan.trajectory.s_coef[2], self.motion_plan.trajectory.s_coef[3],
            #    self.motion_plan.trajectory.s_coef[4], self.motion_plan.trajectory.s_coef[5]
            #))
            #if old_s > self.state.s and not self.motion_plan.reversing:
            #    self.jump_back_count += 1
                #new_pos = np.array([self.state.x, self.state.y])
                #delta_pos = new_pos - old_pos
                #dist = math.sqrt(delta_pos[0]*delta_pos[0] + delta_pos[1]*delta_pos[1])
                #dist = old_s - self.state.s
                #self.max_jump_back_dist = max(self.max_jump_back_dist, dist)

                #log.info("???VID: %d, Plan Tick: %d, Old: %f, Start: %f, New: %f, Diff: %f???" % (
                #    self.id, self.motion_plan.tick_count,
                #    old_s, self.motion_plan.trajectory.get_state_at(0)[0],
                #    self.state.s, diff
                #))

   

    def set_new_motion_plan(self, plan:MotionPlan, sim_time):
        """
        Set a new Motion Plan with a trajectory to start following at start_time.
        """
        if (plan.trajectory.T == 0):
            self.next_motion_plan = None
        else:
            self.next_motion_plan = plan
        
        
        


class EV(Vehicle):
    """
    An external vehicle (remote simulation)
    """
    def __init__(self, vid, name='', start_state=[0.0,0.0,0.0, 0.0,0.0,0.0]):
        super().__init__(vid = vid, name=name, start_state=start_state)
        self.type = Vehicle.EV_TYPE
        self.P = np.identity(2) * 0.5 # some large error        
        # experiment w these values
        self.POS_VAR = 0.01 ** 2
        self.VEL_VAR = 0.1 ** 2
        self.SENSOR_VAR = 0.01 ** 2

    def update_sim_state(self, new_state, delta_time):
        # NOTE: this may desync the sim and frenet vehicle state, so this should
        # only be done for remote vehicles (which don't have a frenet state)

        position, velocity = self.get_kalman_state_estimate(new_state, self.state, delta_time)

        # set filtered velocity but not position - should set pos also?
        self.state.set_X([new_state.x, velocity[0], new_state.x_acc])
        self.state.set_Y([new_state.y, velocity[1], new_state.y_acc])
        # log.info(str(np.linalg.norm([self.state.x_vel, self.state.y_vel])))

    def get_kalman_state_estimate(self, new_state, current_state, delta_time):
        # Init current state as [position velocity]T
        x = np.array([
            [current_state.x, current_state.y],
            [current_state.x_vel, current_state.y_vel]
        ])
        # current observation
        z = np.array([
            [new_state.x, new_state.y]
        ])
        # Init matrices for kalman filter
        # Dynamics matrix
        F = np.array([
            [1, delta_time],
            [0, 1]
        ])
        # Measurement matrix (we only measure position)
        H = np.array([[1, 0]])
        # Error in prediction
        Q = np.array([
            [self.POS_VAR, 0],
            [0, self.VEL_VAR]
        ])
        # Error in measurement
        R = np.array([self.SENSOR_VAR])

        x, self.P = kalman(x, z, self.P, F, H, Q, R)
        return x[0], x[1]

    def get_low_pass_state_estimate(self, new_state, delta_time):
        # simple velocity estimate - not used
        v = np.array([new_state.x_vel, new_state.y_vel])
        v_prev = np.array([self.state.x_vel, self.state.y_vel])
        k = 0.85
        v = (1 - k) * v_prev + k * v
        return [new_state.x, new_state.y], v


class TV(Vehicle):
    """
    A trajectory following vehicle.
    @param keep_active: If True, pedestrian stays in simulation even when is not following a trajectory
    """
    def __init__(self, vid, name, start_state, trajectory, keep_active = True):
        super().__init__(vid, name, start_state)
        self.type = Vehicle.TV_TYPE
        self.trajectory = trajectory
        self.keep_active = keep_active
        if not keep_active:
            #starts as inactive until trajectory begins
            self.sim_state = ActorSimState.INACTIVE 
            self.state.set_X([9999, 0, 0])
            self.state.set_Y([9999,0,0])
        
    def tick(self, tick_count, delta_time, sim_time):
        Vehicle.tick(self, tick_count, delta_time, sim_time)
        self.follow_trajectory(sim_time, self.trajectory)
            
                    


     