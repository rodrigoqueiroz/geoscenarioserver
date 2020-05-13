#!/usr/bin/env python
#rqueiroz@gsd.uwaterloo.ca
#d43sharm@edu.uwaterloo.ca
# --------------------------------------------
# SIMULATED VEHICLES
# --------------------------------------------

from matplotlib import pyplot as plt
import math
import sys
from Utils import *
from SVPlanner import *
from TickSync import TickSync
from VehicleState import *

# A Simulated Vehicle
class SV(object):
    def __init__(self, vid, start_state):
        #id
        self.vid = vid
        #state
        self.vehicle_state = VehicleState()
        self.vehicle_state.x = start_state[0]
        self.vehicle_state.x_vel = start_state[1]
        self.vehicle_state.x_acc = start_state[2]
        self.vehicle_state.y = start_state[3]
        self.vehicle_state.y_vel = start_state[4]
        self.vehicle_state.y_acc = start_state[5]
        #Map
        self.lanelet_map = None
        #Planning
        self.sv_planner = None
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
        #debug temp:
        self.last_x = 0
        self.write_count = 0
    
    def get_sim_state(self):
        #print(self.vehicle_state)
        x = round((self.vehicle_state.x * 100))
        y = round((self.vehicle_state.y * 100))
        z = round((self.vehicle_state.y * 100))
        position = [x, y, z]
        velocity = [self.vehicle_state.x_vel, self.vehicle_state.y_vel]
        return self.vid, position, velocity, self.vehicle_state.yaw, self.vehicle_state.steer
    
    def set_behavior_root(self, btree, target_id = None, goal_pos = None):
        """ btree: is the root behavior tree. By default, the tree is velocity keeping.
            target: can be used in many subtrees to force a maneuver towards one specific vehicle. 
                    Usually, Ego is the target. Only one target is allowed.
                    Without a defined target, maneuvers will adapt to vehicles around them as target.
        """
        self.btree = btree
        self.target_id = target_id

    def start_planner(self, nvehicles, laneletmap, traffic_state_sharr ): #,lock_vs, shm_vs, lock_vp, shm_vp):
        """For simulated vehicles controlled by SVPlanner.
            If a planner is started, the vehicle can't have a remote.
            Use either option (start_planner or start_remote).
        """
        self.sv_planner = SVPlanner(self.vid, nvehicles, laneletmap ,traffic_state_sharr) # lock_vs, shm_vs, lock_vp, shm_vp)
        self.sv_planner.start()
    
    def start_remote(self):
        """For vehicles controlled by external source. 
            For example, Ego/Subject vehicle, or a vehicle controlled by Tester
            If a remote is started, the vehicle can't have a planner
            Use either option (start_planner or start_remote).
        """
        pass

    def tick(self, tick_count, delta_time, sim_time):
        if (self.trajectory):
            self.trajectory_time += delta_time
            self.compute_vehicle_state(self.trajectory_time)

        #Read output from planner, if available
        plan = self.sv_planner.get_plan()
        if (plan):
            print('NEW PLAN')
            self.set_new_trajectory( plan.get_trajectory() )
            #advance trajectory progress based 
            #on difference between planning time and now
            print('Plan start {}, now is {}'.format(plan.t_start, sim_time))
            diff = sim_time - plan.t_start
            if (diff>0):
                print('Advance traj in {} s'.format(diff))
                self.trajectory_time += diff

 
    def compute_vehicle_state(self,time):
        """
        Consume trajectory based on a given time and update pose
        Optimized with pre computed derivatives and equations
        """
        if (time>self.trajectory[2]): #exceed total traj time
            return

        self.vehicle_state.x = self.s_eq(time)
        self.vehicle_state.x_vel = self.s_vel_eq(time)
        self.vehicle_state.x_acc = self.s_acc_eq(time)
        self.vehicle_state.y = self.d_eq(time)
        self.vehicle_state.y_vel = self.d_vel_eq(time)
        self.vehicle_state.y_acc = self.d_acc_eq(time)
        #sanity check
        if ( self.vehicle_state.x < self.last_x) :
            diff = self.vehicle_state.x - self.last_x
            print ('ERROR: X went backwards. Diff: {:.2}'.format(diff))
        self.last_x = self.vehicle_state.x


    def set_new_trajectory(self, trajectory, candidates = None):
        """
        Set a new trajectory to start following immediately.
        candidates: save computed trajectories for visualization and debug
        """
        s_coef,d_coef,_ = trajectory
        self.trajectory = trajectory
        self.trajectory_time = 0 
        self.cand_trajectories = candidates
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
      
    def state_in(self, t):
        """
        Predicts a new state based on time along the active trajectory.
        Used for collision prediction and charts
        """
        if (self.trajectory):
            t = self.trajectory_time + t
            state = [
                self.s_eq(t),
                self.s_vel_eq(t),
                self.s_acc_eq(t),
                self.d_eq(t),
                self.d_vel_eq(t),
                self.d_acc_eq(t),
            ] 
        else:
            #If no trajectory, returns current state
            state = [
                self.s_pos, 
                self.s_vel, 
                self.s_acc, 
                self.d_pos, 
                self.d_vel, 
                self.d_acc
            ]
        return state
    
 