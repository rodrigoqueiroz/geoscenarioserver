#!/usr/bin/env python
#rqueiroz@uwaterloo.ca
#d43sharm@uwaterloo.ca
# --------------------------------------------
# SIMULATED VEHICLES
# --------------------------------------------

from matplotlib import pyplot as plt
import math
import sys
import glog as log
import numpy as np
from TickSync import TickSync
from SimConfig import *
from util.Transformations import frenet_to_sim_frame, sim_to_frenet_frame, OutsideRefPathException
from util.Utils import *
from sv.SVPlanner import *
from sv.VehicleState import *
from mapping.LaneletMap import LaneletMap
from shm.SimSharedMemory import *
from util.Utils import kalman


# Vehicle base class for remote control or simulation.
class Vehicle(object):
    #vehicle types
    SDV_TYPE = 0
    RV_TYPE = 1
    TV_TYPE = 2

    #vehicle simulation modes
    INACTIVE = 0          #vehicle not in simulation, not present in traffic, and not visible for other vehicles
    ACTIVE = 1            #vehicle is in simulation and visible to other vehicles
    INVISIBLE = 2         #vehicle is in simulation but NOT visible to other vehicles (for reference)

    def __init__(self, vid, type, name='', btree_root='', start_state=[0.0,0.0,0.0, 0.0,0.0,0.0], frenet_state=[0.0,0.0,0.0, 0.0,0.0,0.0], radius=VEHICLE_RADIUS, model=None):
        #id
        self.vid = vid
        self.name = name
        self.radius = radius
        self.model = model
        self.type = type

        self.sim_state = Vehicle.ACTIVE
        
        #state
        #start state in sim frame
        self.vehicle_state = VehicleState()
        self.vehicle_state.x = start_state[0]
        self.vehicle_state.x_vel = start_state[1]
        self.vehicle_state.x_acc = start_state[2]
        self.vehicle_state.y = start_state[3]
        self.vehicle_state.y_vel = start_state[4]
        self.vehicle_state.y_acc = start_state[5]
        # start state in frenet
        self.vehicle_state.s = frenet_state[0]
        self.vehicle_state.s_vel = frenet_state[1]
        self.vehicle_state.s_acc = frenet_state[2]
        self.vehicle_state.d = frenet_state[3]
        self.vehicle_state.d_vel = frenet_state[4]
        self.vehicle_state.d_acc = frenet_state[5]

                


    def future_state(self, t):
        """ Predicts a new state based on time and vel.
            Used for collision prediction and charts
            TODO: predict using history
        """
        state = [
            self.vehicle_state.s + (self.vehicle_state.s_vel * t) + (self.vehicle_state.s_acc * t * t),
            self.vehicle_state.s_vel + (self.vehicle_state.s_acc * t),
            self.vehicle_state.s_acc,
            self.vehicle_state.d + (self.vehicle_state.d_vel * t) + (self.vehicle_state.d_acc * t * t),
            self.vehicle_state.d_vel + (self.vehicle_state.d_acc * t),
            self.vehicle_state.d_acc
        ]
        return state

    def update_sim_state(self, new_state, delta_time):
        # NOTE: this may desync the sim and frenet vehicle state, so this should
        # only be done for remote vehicles (which don't have a frenet state)
        if self.type is not Vehicle.RV_TYPE:
            log.warn("Cannot update sim state for gs vehicles directly.")

    def stop(self):
        pass

    def remove(self):
        self.sim_state = Vehicle.INACTIVE

    def tick(self, tick_count, delta_time, sim_time):
        pass

    def get_sim_state(self):
        x = round((self.vehicle_state.x * CLIENT_METER_UNIT))
        y = round((self.vehicle_state.y * CLIENT_METER_UNIT))
        z = round((self.vehicle_state.y * CLIENT_METER_UNIT))
        position = [x, y, z]
        velocity = [self.vehicle_state.x_vel, self.vehicle_state.y_vel]
        #remote = 1 if self.is_remote else 0
        return self.vid, type, position, velocity, self.vehicle_state.yaw, self.vehicle_state.steer


# A Simulated Driver-Vehicle (dynamic behavior)
class SV(Vehicle):
    def __init__(self, vid, name, btree_root, start_state, radius, lanelet_map, lanelet_route, start_state_in_frenet=False, model=None):
        #Map
        self.lanelet_map = lanelet_map
        # list of lanelet ids we want this vehicle to follow
        self.lanelet_route = lanelet_route
        self.global_path = None
        
        if start_state_in_frenet:
            # assume frenet start_state is relative to the first lane of the route
            self.global_path = self.lanelet_map.get_global_path_for_route(self.lanelet_route)
            # compute sim state
            x_vector, y_vector = frenet_to_sim_frame(self.global_path, start_state[0:3], start_state[3:])
            Vehicle.__init__(self, vid, Vehicle.SDV_TYPE, name, start_state=(x_vector+y_vector), frenet_state=start_state, radius=radius, model=model)
        else:
            self.global_path = self.lanelet_map.get_global_path_for_route(self.lanelet_route, x=start_state[0], y=start_state[3])
            # Compute frenet state corresponding to start_state
            s_vector, d_vector = sim_to_frenet_frame(self.global_path, start_state[0:3], start_state[3:])
            Vehicle.__init__(self, vid, Vehicle.SDV_TYPE, name, start_state=start_state, frenet_state=(s_vector + d_vector), radius=radius, model=model)
        # print("{} init'd with {}, {}".format(vid, self.vehicle_state.s, self.vehicle_state.d))

        #Planning
        self.sv_planner = None
        #behavior
        self.btree_root = btree_root
        #Trajectory
        self.trajectory = None          #coefs + time[[0,0,0,0,0,0],[0,0,0,0,0,0],[0]]
        self.trajectory_time = 0        #consumed time
        self.cand_trajectories = None   #plotting only
        self.s_eq = None
        self.s_vel_eq = None
        self.s_acc_eq = None
        self.d_eq = None
        self.d_vel_eq = None
        self.d_acc_eq = None
        self.reversing = False

    # def future_state(self, t):
    #     """ Predicts a new state based on time
    #         and current trajectory *override
    #     """
    #     if (not self.trajectory): #If no trajectory, returns prediction
    #         return Vehicle.future_state(self,t)

    #     t = self.trajectory_time + t
    #     state = [
    #         self.s_eq(t),
    #         self.s_vel_eq(t),
    #         self.s_acc_eq(t),
    #         self.d_eq(t),
    #         self.d_vel_eq(t),
    #         self.d_acc_eq(t),
    #     ]
    #     return state

    def start_planner(
            self,
            nvehicles,
            sim_config,
            traffic_state_sharr,
            traffic_light_sharr,
            debug_shdata):
        """For SDV models controlled by SVPlanner.
            If a planner is started, the vehicle can't be a remote.
        """
        self.sv_planner = SVPlanner(
            self.vid, self.btree_root, nvehicles, self.lanelet_map, sim_config,
            traffic_state_sharr, traffic_light_sharr, debug_shdata
        )
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
                if plan.t != 0:
                    self.set_new_motion_plan(plan, sim_time)
                    if plan.new_frenet_frame:
                        # NOTE: vehicle state being used here is from the pervious frame (that planner should have gotten)
                        self.global_path = self.lanelet_map.get_global_path_for_route(self.lanelet_route, self.vehicle_state.x, self.vehicle_state.y)
                else:
                    self.set_new_motion_plan(plan, sim_time)

            #Compute new state
            self.compute_vehicle_state(delta_time)

    def compute_vehicle_state(self, delta_time):
        """
        Consume trajectory based on a given time and update pose
        Optimized with pre computed derivatives and equations
        """
        if (self.trajectory):
            self.trajectory_time += delta_time
            time = self.trajectory_time

            if (time > self.trajectory[2]): #exceed total traj time
                return

            # sanity check
            # if self.s_eq(time) < self.vehicle_state.s:
            #     print ('ERROR: S went backwards from {:.2} to {:.2}'.format(self.vehicle_state.s, self.s_eq(time)))
            #     raise Exception()
            # else:
            #     print ('S: {:.2}'.format(self.s_eq(time)))

            # update frenet state
            self.vehicle_state.s = self.s_eq(time)
            self.vehicle_state.s_vel = self.s_vel_eq(time)
            self.vehicle_state.s_acc = self.s_acc_eq(time)
            self.vehicle_state.d = self.d_eq(time)
            self.vehicle_state.d_vel = self.d_vel_eq(time)
            self.vehicle_state.d_acc = self.d_acc_eq(time)

            # Compute sim state using the global path this vehicle is following
            # self.global_path = self.lanelet_map.get_global_path_for_route(self.vehicle_state.x, self.vehicle_state.y, self.lanelet_route)
            try:
                x_vector, y_vector = frenet_to_sim_frame(self.global_path, self.vehicle_state.get_S(), self.vehicle_state.get_D())
            except OutsideRefPathException as e:
                # assume we've reached the goal and exit?
                raise e

            # update sim state
            self.vehicle_state.x = x_vector[0]
            self.vehicle_state.x_vel = x_vector[1]
            self.vehicle_state.x_acc = x_vector[2]
            self.vehicle_state.y = y_vector[0]
            self.vehicle_state.y_vel = y_vector[1]
            self.vehicle_state.y_acc = y_vector[2]
            heading = np.array([y_vector[1], x_vector[1]])
            if self.reversing:
                heading *= -1
            self.vehicle_state.yaw = math.degrees(math.atan2(heading[1], heading[0]))
            #sanity check
            #if ( self.vehicle_state.x < self.last_x) :
            #    diff = self.vehicle_state.x - self.last_x
            #    print ('ERROR: X went backwards. Diff: {:.2}'.format(diff))
            #self.last_x = self.vehicle_state.x

    def get_frenet_state(self):
        if self.s_eq is None:
            return None
        time = self.trajectory_time
        return [self.s_eq(time), self.s_vel_eq(time), self.s_acc_eq(time), self.d_eq(time), self.d_vel_eq(time), self.d_acc_eq(time)]

    def get_global_path(self):
        """ The ref path the SV is following
        """
        pass

    def set_new_motion_plan(self, plan, sim_time):
        """
        Set a new trajectory to start following immediately.
        candidates: save computed trajectories for visualization and debug
        """
        if (plan.t == 0):
            self.trajectory = None
            return

        # if self.s_eq:
        #     print('current s {} at t {}'.format(self.s_eq(self.trajectory_time), self.trajectory_time))
        trajectory = plan.get_trajectory()

        s_coef,d_coef,_ = trajectory
        self.trajectory = trajectory
        self.trajectory_time = 0
        #self.cand_trajectories = candidates
        #Pre-compute derivatives and equations for execution
        #S
        self.s_eq = to_equation(s_coef)
        s_vel_coef = differentiate(s_coef)
        self.s_vel_eq = to_equation(s_vel_coef)
        s_acc_coef = differentiate(s_vel_coef)
        self.s_acc_eq = to_equation(s_acc_coef)
        #D
        self.d_eq = to_equation(d_coef)
        d_vel_coef = differentiate(d_coef)
        self.d_vel_eq = to_equation(d_vel_coef)
        d_acc_coef = differentiate(d_vel_coef)
        self.d_acc_eq = to_equation(d_acc_coef)

        self.reversing = plan.reversing

        #Correct trajectory progress based
        #on diff planning time and now
        diff = sim_time - plan.t_start
        # print('Plan start {}, now is {}'.format(plan.t_start, sim_time))
        # print('Advance traj in {} s'.format(diff))
        if (diff > 0):
            self.trajectory_time += diff
        # print('new s {} at t {}'.format(self.s_eq(self.trajectory_time), self.trajectory_time))

#A remote vehicle
class RV(Vehicle):
    def __init__(self, vid, name='', btree_root='', start_state=[0.0,0.0,0.0, 0.0,0.0,0.0], radius=VEHICLE_RADIUS):
        super(RV, self).__init__(vid, Vehicle.RV_TYPE, name=name, start_state=start_state, radius=radius)
        self.P = np.identity(2) * 0.5 # some large error
        # experiment w these values
        self.POS_VAR = 0.01 ** 2
        self.VEL_VAR = 0.1 ** 2
        self.SENSOR_VAR = 0.01 ** 2

    def update_sim_state(self, new_state, delta_time):
        # NOTE: this may desync the sim and frenet vehicle state, so this should
        # only be done for remote vehicles (which don't have a frenet state)

        position, velocity = self.get_kalman_state_estimate(new_state, self.vehicle_state, delta_time)

        # set filtered velocity but not position - should set pos also?
        self.vehicle_state.set_X([new_state.x, velocity[0], new_state.x_acc])
        self.vehicle_state.set_Y([new_state.y, velocity[1], new_state.y_acc])
        # log.info(str(np.linalg.norm([self.vehicle_state.x_vel, self.vehicle_state.y_vel])))

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
        v_prev = np.array([self.vehicle_state.x_vel, self.vehicle_state.y_vel])
        k = 0.85
        v = (1 - k) * v_prev + k * v
        return [new_state.x, new_state.y], v

#A trajectory following vehicle
class TV(Vehicle):
    def __init__(self, vid, name, start_state, trajectory, lanelet_map, ghost_mode = False,radius=VEHICLE_RADIUS):
        super(TV, self).__init__(vid, Vehicle.TV_TYPE, name, start_state, radius=radius)
        self.lanelet_map = lanelet_map
        self.trajectory = trajectory
        self.sim_state = Vehicle.INACTIVE #starts as inactive until trajectory begins
        self.vehicle_state.set_X([9999, 0, 0]) #forcing
        self.vehicle_state.set_Y([9999,0,0])
        self.ghost_mode = ghost_mode

    def tick(self, tick_count, delta_time, sim_time):
        Vehicle.tick(self, tick_count, delta_time, sim_time)
            
        if self.trajectory:
            start_time = float(self.trajectory[0].time) #first node
            end_time = float(self.trajectory[-1].time) #last node
            traj_time = end_time - start_time
            #During trajectory
            if start_time <= sim_time <= end_time:
                #for node in self.trajectory:
                for i in range(len(self.trajectory)):
                    node = self.trajectory[i]
                    if node.time < sim_time:
                        continue
                    #closest after current sim time
                    #TODO: interpolate taking the difference between closest node time and sim time
                    #print("closest node time {} >= simtime {}".format(node_time,sim_time))
                    if self.sim_state is Vehicle.INACTIVE:
                        log.warn("vid {} is now ACTIVE".format(self.vid))
                        self.sim_state = Vehicle.ACTIVE
                        if self.ghost_mode:
                            self.sim_state = Vehicle.INVISIBLE
                            log.warn("vid {} is now INVISIBLE".format(self.vid))
                            #workaround for evaluation only
                            if -self.vid in self.simtraffic.vehicles:
                                self.simtraffic.vehicles[-self.vid].sim_state = Vehicle.ACTIVE 
                    #Update Vehicle State
                    if (i>1):
                        prevnode =  self.trajectory[i-1]
                        xvel = (node.x - prevnode.x) / (node.time - prevnode.time)
                        yvel = (node.y - prevnode.y) / (node.time - prevnode.time)
                    else:
                        xvel = yvel = 0
                    xacc = yacc = 0 #todo
                    self.vehicle_state.set_X([node.x, xvel, xacc])
                    self.vehicle_state.set_Y([node.y, yvel, xacc])
                    break
            #After trajectory
            if sim_time > end_time:
                #vanish. Need to optimize this by setting vehicles as not visible and removing all calculations with it
                self.vehicle_state.set_X([-9999, 0, 0])
                self.vehicle_state.set_Y([-9999,0,0])
                if self.sim_state is Vehicle.ACTIVE or self.sim_state is Vehicle.INVISIBLE:
                    log.warn("vid {} is now INACTIVE".format(self.vid))
                    self.sim_state = Vehicle.INACTIVE
                    #workaround for evaluation only
                    if -self.vid in self.simtraffic.vehicles:
                        self.simtraffic.vehicles[-self.vid].sim_state = Vehicle.INACTIVE 
                
            
        

    
