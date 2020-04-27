#!/usr/bin/env python
#rqueiroz@gsd.uwaterloo.ca
#d43sharm@edu.uwaterloo.ca
# --------------------------------------------
# SIMULATED HUMAN CONTROLLED VEHICLE AND FRENET PATH
# --------------------------------------------

from matplotlib import pyplot as plt
from Utils import *
from SVPlanner import *
from TickSync import TickSync
import math
import threading

# A Simulated Vehicle
class SV(object):
    
    def __init__(self, id, start_state):
        #id
        self.id = id
       
        #Mission in sim frame
        self.xgoal = 0
        self.ygoal = 0

        #Map
        self.map_server = None
        self.lanelet_map = None

        #Planning
        self.sv_planner = None

        self.lookahead_dist = 0
        self.trajectory = None          #coefs + total time[[0,0,0,0,0,0],[0,0,0,0,0,0],[0]] 
        self.trajectory_time = 0        #consumed time
        self.cand_trajectories = None   #plotting only
        self.s_eq = None
        self.s_vel_eq = None
        self.s_acc_eq = None
        self.d_eq = None
        self.d_vel_eq = None
        self.d_acc_eq = None

        #state in sim frame / sync with simulator
        self.last_x = 0
        self.write_count = 0
        self.x = 0
        self.y = 0
        self.z = 0
        self.yaw = 0
        self.steering_angle = 0
        self.vel = 0
        self.acc = 0
        self.x_vel = 0
        self.y_vel = 0
        #state in moving frenet frame
        self.s_pos = start_state[0]
        self.s_vel = start_state[1]
        self.s_acc = start_state[2]
        self.d_pos = start_state[3]
        self.d_vel = start_state[4]
        self.d_acc = start_state[5]
  
    def get_stats(self):
        vehicle_state =  [self.s_pos, self.s_vel, self.s_acc, self.d_pos, self.d_vel, self.d_acc]
        return self.id, vehicle_state, self.trajectory, self.cand_trajectories

    def get_sim_state(self):
        position = [self.x, self.y, self.z]
        velocity = [self.x_vel, self.y_vel]
        return self.id, position, velocity, self.yaw, self.steering_angle
    
    def set_behavior_root(self, btree, target_id = None, goal_pos = None):
        """ btree: is the root behavior tree. By default, the tree is velocity keeping.
            target: can be used in many subtrees to force a maneuver towards one specific vehicle. 
                    Usually, Ego is the target. Only one target is allowed.
                    Without a defined target, maneuvers will adapt to vehicles around them as target.
        """
        self.btree = btree
        self.target_id = target_id
    
    def set_environment(self, vehicles, lanelet_map):
        """ Set environment with a list of other vehicles and a map. 
            If no environment is available, vehicle will run the simulation alone in a virtual straight road.
        """
        #TODO
        pass

    def start_vehicle(self):
        #start map server
        #self.map_server = MapServer(self.lanelet_map)
        #start planner
        self.sv_planner = SVPlanner(self.map_server)
        self.sv_planner.start_planner()

    def tick(self, tick_count, delta_time):
        
        if (self.trajectory):
            self.trajectory_time += delta_time
            self.compute_vehicle_state(self.trajectory_time)

        #Road Config
        #todo: Get road attributes from laneletmap. Hardcoding now.
        lane_config = LaneConfig(30,4,0)
        
        vehicle_state =  [self.s_pos, self.s_vel, self.s_acc, self.d_pos, self.d_vel, self.d_acc]
        

        #Dynamic Traffic: Vehicles/Pedestrians/Animals/Obstacles
        trafic_state = None #TODO

        #Frame snapshot
        self.sv_planner.update_snapshot(vehicle_state,lane_config, trafic_state)

        #Read output from planner, if available
        #renew_plans = self.svplanner.run_planner(vehicle_state,lane_config, trafic_state)
        new_plan = self.sv_planner.get_plan()
        if (new_plan):
            traj, cand = new_plan
            self.set_new_trajectory(traj,cand)
 
    def compute_vehicle_state(self,time):
        """
        Consume trajectory based on a given time and update pose
        Optimized with pre computed derivatives and equations
        """
        self.s_pos = self.s_eq(time)
        self.s_vel = self.s_vel_eq(time)
        self.s_acc = self.s_acc_eq(time)
        self.d_pos = self.d_eq(time)
        self.d_vel = self.d_vel_eq(time)
        self.d_acc = self.d_acc_eq(time)
        
        self.x = round((self.s_pos * 100))
        self.y = round((self.d_pos * 100))
        #self.z = ?

        self.x_vel = self.s_vel
        self.y_vel = self.y_vel

        #self.yaw = ?
        self.steering_angle = 0


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
    
 