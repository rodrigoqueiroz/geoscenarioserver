#!/usr/bin/env python
#rqueiroz@gsd.uwaterloo.ca
#d43sharm@uwaterloo.ca
# --------------------------------------------
# SIMULATED VEHICLES
# --------------------------------------------

from matplotlib import pyplot as plt
import math
import sys
from TickSync import TickSync
from util.Utils import *
from util.Constants import *
from sv.SVPlanner import *
from sv.VehicleState import *
from Mapping.LaneletMap import LaneletMap, OutsideRefPathException

from shm.SimSharedMemory import *

# Vehicle base class for remote control or simulation.
class Vehicle(object):
    def __init__(self, vid, name = '', btree_root = '', start_state = [0.0,0.0,0.0, 0.0,0.0,0.0], frenet_state = [0.0,0.0,0.0, 0.0,0.0,0.0], radius = VEHICLE_RADIUS, model = None):
        #id
        self.vid = vid
        self.name = name
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
        #stats
        self.radius = radius
        self.model = model
        #remote
        self.is_remote = False
        

    def future_state(self, t):
        """ Predicts a new state based on time and vel.
            Used for collision prediction and charts
            TODO: predict using history
            NOTE: do we change this to use frenet?
        """
        state = [
            self.vehicle_state.s + (self.vehicle_state.s_vel * t),
            self.vehicle_state.s_vel, 
            self.vehicle_state.s_acc, 
            self.vehicle_state.d + (self.vehicle_state.d_vel * t), 
            self.vehicle_state.d_vel, 
            self.vehicle_state.d_acc
        ]
        return state
    
    def stop(self):
        pass

    def tick(self, tick_count, delta_time, sim_time):
        pass
    
    def get_sim_state(self):
        x = round((self.vehicle_state.x * CLIENT_METER_UNIT))
        y = round((self.vehicle_state.y * CLIENT_METER_UNIT))
        z = round((self.vehicle_state.y * CLIENT_METER_UNIT))
        position = [x, y, z]
        velocity = [self.vehicle_state.x_vel, self.vehicle_state.y_vel]
        remote = 1 if self.is_remote else 0
        return self.vid, remote, position, velocity, self.vehicle_state.yaw, self.vehicle_state.steer
    

# A Simulated Vehicle
class SV(Vehicle):
    def __init__(self, vid, name, btree_root, start_state, radius, lanelet_map, lanelet_route, start_state_in_frenet=False, model = None):
        #Map
        self.lanelet_map = lanelet_map
        # list of lanelet ids we want this vehicle to follow
        self.lanelet_route = lanelet_route
        self.global_path = None

        if start_state_in_frenet:
            # assume frenet start_state is relative to the first lane of the route
            self.global_path = self.lanelet_map.get_global_path_for_route(self.lanelet_route)
            # compute sim state
            x_vector, y_vector = self.lanelet_map.frenet_to_sim_frame(self.global_path, start_state[0:3], start_state[3:])
            Vehicle.__init__(self, vid, name, start_state=(x_vector+y_vector), frenet_state=start_state, radius=radius, model=model)
        else:
            self.global_path = self.lanelet_map.get_global_path_for_route(self.lanelet_route, x=start_state[0], y=start_state[3])
            # Compute frenet state corresponding to start_state
            s_vector, d_vector = self.lanelet_map.sim_to_frenet_frame(self.global_path, start_state[0:3], start_state[3:])
            Vehicle.__init__(self, vid, name, start_state=start_state, frenet_state=(s_vector + d_vector), radius=radius, model=model)
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

    def future_state(self, t):
        """ Predicts a new state based on time 
            and current trajectory *override
        """
        if (not self.trajectory): #If no trajectory, returns prediction
            return Vehicle.future_state(self,t)

        t = self.trajectory_time + t
        state = [
            self.s_eq(t),
            self.s_vel_eq(t),
            self.s_acc_eq(t),
            self.d_eq(t),
            self.d_vel_eq(t),
            self.d_acc_eq(t),
        ] 
        return state
    
        
    def start_planner(self, nvehicles, sim_config, traffic_state_sharr ):
        """For simulated vehicles controlled by SVPlanner.
            If a planner is started, the vehicle can't have a remote.
            Use either option (start_planner or start_remote).
        """
        self.is_remote = False
        self.sv_planner = SVPlanner(self.vid, self.btree_root, nvehicles, self.lanelet_map, sim_config, traffic_state_sharr)
        self.sv_planner.start()
    
    def stop(self):
        if self.sv_planner:
            self.sv_planner.stop()

    def tick(self, tick_count, delta_time, sim_time):
        Vehicle.tick(self, tick_count, delta_time, sim_time)

        #Read planner
        plan = self.sv_planner.get_plan()
        if (plan):
            self.set_new_motion_plan(plan, sim_time)
            if plan.new_frenet_frame:
                # NOTE: vehicle state being used here is from the pervious frame (that planner should have gotten)
                self.global_path = self.lanelet_map.get_global_path_for_route(self.lanelet_route, self.vehicle_state.x, self.vehicle_state.y)
        
        #Compute new state
        self.compute_vehicle_state(delta_time)

    def compute_vehicle_state(self,delta_time):
        """
        Consume trajectory based on a given time and update pose
        Optimized with pre computed derivatives and equations
        """
        if (self.trajectory):
            self.trajectory_time += delta_time
            time = self.trajectory_time

            if (time>self.trajectory[2]): #exceed total traj time
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
            # TODO: rn the global path isn't changing - its using the centerline of the entire route. needs to change cause lane changes
            # self.global_path = self.lanelet_map.get_global_path_for_route(self.vehicle_state.x, self.vehicle_state.y, self.lanelet_route)
            try:
                x_vector, y_vector = self.lanelet_map.frenet_to_sim_frame(self.global_path, self.vehicle_state.get_S(), self.vehicle_state.get_D())
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
            #sanity check
            #if ( self.vehicle_state.x < self.last_x) :
            #    diff = self.vehicle_state.x - self.last_x
            #    print ('ERROR: X went backwards. Diff: {:.2}'.format(diff))
            #self.last_x = self.vehicle_state.x
    

    def get_frenet_state(self):
        if self.s_eq == None:
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

        #Correct trajectory progress based 
        #on diff planning time and now
        diff = sim_time - plan.t_start
        # print('Plan start {}, now is {}'.format(plan.t_start, sim_time))
        # print('Advance traj in {} s'.format(diff))
        if (diff>0):
            self.trajectory_time += diff
        # print('new s {} at t {}'.format(self.s_eq(self.trajectory_time), self.trajectory_time))
            
   
    
 