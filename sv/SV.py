#!/usr/bin/env python
#rqueiroz@gsd.uwaterloo.ca
#d43sharm@edu.uwaterloo.ca
# --------------------------------------------
# SIMULATED VEHICLES
# --------------------------------------------

from matplotlib import pyplot as plt
import math
import sys
from TickSync import TickSync
from util.Utils import *
from sv.SVPlanner import *
from sv.VehicleState import *
from shared_mem.EgoSharedMemory import *
from Mapping.LaneletMap import LaneletMap


# Vehicle base class for remote control or simulation.
class Vehicle(object):
    def __init__(self, vid, name = '', start_state = [0.0,0.0,0.0, 0.0,0.0,0.0], radius = VEHICLE_RADIUS, model = None):
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
        #stats
        self.radius = radius
        self.model = model
        #remote
        self.is_remote = False
        self._remote_shared_memory = None
    
    def start_remote(self):
        """ For vehicles controlled by external source. 
            For example: Ego/Subject vehicle, or a vehicle controlled by a Tester
            If a remote is started, the vehicle can't have a planner
            Use either option (start_planner or start_remote).
        """
        self._remote_shared_memory = EgoSharedMemory()
        
    def stop(self):
        self._remote_shared_memory = None

    def future_state(self, t):
        """ Predicts a new state based on time and vel.
            Used for collision prediction and charts
            TODO: predict using history
        """
        state = [
            self.vehicle_state.x + (self.vehicle_state.x_vel * t),
            self.vehicle_state.x_vel, 
            self.vehicle_state.x_acc, 
            self.vehicle_state.y + (self.vehicle_state.y_vel * t), 
            self.vehicle_state.y_vel, 
            self.vehicle_state.y_acc
        ]
        return state
       
    def tick(self, tick_count, delta_time, sim_time):
        if self.is_remote:
            if _remote_shared_memory.is_connected:
                self._remote_shared_memory.read_memory()
                self._remote_shared_memory
    
    def get_sim_state(self):
        x = round((self.vehicle_state.x * 100))
        y = round((self.vehicle_state.y * 100))
        z = round((self.vehicle_state.y * 100))
        position = [x, y, z]
        velocity = [self.vehicle_state.x_vel, self.vehicle_state.y_vel]
        return self.vid, position, velocity, self.vehicle_state.yaw, self.vehicle_state.steer
    

# A Simulated Vehicle
class SV(Vehicle):
    def __init__(self, vid, name, start_state, radius, model = None):
        Vehicle.__init__(self, vid, name, start_state, radius, model)
        #Map
        self.lanelet_map = None
        # list of lanelet ids we want this vehicle to follow
        self.lanelet_route = None
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
    
    def set_behavior_root(self, btree, target_id = None, goal_pos = None):
        """ btree: is the root behavior tree. By default, the tree is velocity keeping.
            target: can be used in many subtrees to force a maneuver towards one specific vehicle. 
                    Usually, Ego is the target. Only one target is allowed.
                    Without a defined target, maneuvers will adapt to vehicles around them as target.
        """
        self.btree = btree
        self.target_id = target_id

    def start_planner(self, nvehicles, laneletmap, traffic_state_sharr ):
        """For simulated vehicles controlled by SVPlanner.
            If a planner is started, the vehicle can't have a remote.
            Use either option (start_planner or start_remote).
        """
        self.lanelet_map = laneletmap # move this to init?
        self.is_remote = False
        self.sv_planner = SVPlanner(self.vid, nvehicles, laneletmap ,traffic_state_sharr)
        self.sv_planner.start()
    
    def stop(self):
        if self.sv_planner:
            self.sv_planner.stop()

   
    def tick(self, tick_count, delta_time, sim_time):
        Vehicle.tick(self, tick_count, delta_time, sim_time)
        
        #Compute new state
        self.compute_vehicle_state(delta_time)

        #Read planner
        plan = self.sv_planner.get_plan()
        if (plan):
            self.set_new_motion_plan(plan, sim_time)

 
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

            # update frenet state
            self.vehicle_state.s = self.s_eq(time)
            self.vehicle_state.s_vel = self.s_vel_eq(time)
            self.vehicle_state.s_acc = self.s_acc_eq(time)
            self.vehicle_state.d = self.d_eq(time)
            self.vehicle_state.d_vel = self.d_vel_eq(time)
            self.vehicle_state.d_acc = self.d_acc_eq(time)

            # Compute sim state using the global path this vehicle is following
            global_path = self.lanelet_map.get_global_path_for_route(self.vehicle_state.x, self.vehicle_state.y, self.lanelet_route)
            x, y = LaneletMap.frenet_to_sim_frame(global_path, self.s_eq(time), self.d_eq(time))
            # can we just write out s_eq and d_eq to planner??

            self.vehicle_state.x = x
            self.vehicle_state.x_vel = self.s_vel_eq(time)
            self.vehicle_state.x_acc = self.s_acc_eq(time)
            self.vehicle_state.y = y
            self.vehicle_state.y_vel = self.d_vel_eq(time)
            self.vehicle_state.y_acc = self.d_acc_eq(time)
            #sanity check
            #if ( self.vehicle_state.x < self.last_x) :
            #    diff = self.vehicle_state.x - self.last_x
            #    print ('ERROR: X went backwards. Diff: {:.2}'.format(diff))
            #self.last_x = self.vehicle_state.x
    

    def get_frenet_state(self):
        # ll = self.lanelet_map.get_occupying_lanelet(self.vehicle_state.x, self.vehicle_state.y)
        # s, d = LaneletMap.sim_to_frenet_frame(ll, self.vehicle_state.x, self.vehicle_state.y)
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
        #print('Plan start {}, now is {}'.format(plan.t_start, sim_time))
        #print('Advance traj in {} s'.format(diff))
        diff = sim_time - plan.t_start
        if (diff>0):
            self.trajectory_time += diff
            
   
    
 