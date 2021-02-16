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
from sv.SDVPlanner import *
from Actor import *
from mapping.LaneletMap import LaneletMap
from shm.SimSharedMemory import *
from util.Utils import kalman


# Vehicle base class for remote control or simulation.
class Vehicle(Actor):
    #vehicle types
    N_TYPE = 0  #neutral
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
        return self.id, self.type, position, velocity, self.state.angle, self.state.steer


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
        #Trajectory
        self.trajectory = None          #coefs + time[[0,0,0,0,0,0],[0,0,0,0,0,0],[0]]
        self.trajectory_time = 0        #consumed time
        self.cand_trajectories = None   #plotting only
        self.unf_trajectories = None   #plotting only
        self.s_eq = None
        self.s_vel_eq = None
        self.s_acc_eq = None
        self.d_eq = None
        self.d_vel_eq = None
        self.d_acc_eq = None
        self.reversing = False


    def start_planner(self):
            #nvehicles,
            #,
            #traffic_state_sharr,
            #traffic_light_sharr,
            #debug_shdata):
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
                if plan.t != 0:
                    self.set_new_motion_plan(plan, sim_time)
                    if plan.new_frenet_frame:
                        # NOTE: vehicle state being used here is from the pervious frame (that planner should have gotten)
                        self.global_path = self.lanelet_map.get_global_path_for_route(self.lanelet_route, self.state.x, self.state.y)
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
            # if self.s_eq(time) < self.state.s:
            #     print ('ERROR: S went backwards from {:.2} to {:.2}'.format(self.state.s, self.s_eq(time)))
            #     raise Exception()
            # else:
            #     print ('S: {:.2}'.format(self.s_eq(time)))

            # update frenet state
            self.state.s = self.s_eq(time)
            self.state.s_vel = self.s_vel_eq(time)
            self.state.s_acc = self.s_acc_eq(time)
            self.state.d = self.d_eq(time)
            self.state.d_vel = self.d_vel_eq(time)
            self.state.d_acc = self.d_acc_eq(time)

            # Compute sim state using the global path this vehicle is following
            # self.global_path = self.lanelet_map.get_global_path_for_route(self.state.x, self.state.y, self.lanelet_route)
            try:
                x_vector, y_vector = frenet_to_sim_frame(self.global_path, self.state.get_S(), self.state.get_D())
            except OutsideRefPathException as e:
                # assume we've reached the goal and exit?
                raise e

            # update sim state
            self.state.x = x_vector[0]
            self.state.x_vel = x_vector[1]
            self.state.x_acc = x_vector[2]
            self.state.y = y_vector[0]
            self.state.y_vel = y_vector[1]
            self.state.y_acc = y_vector[2]
            heading = np.array([y_vector[1], x_vector[1]])
            if self.reversing:
                heading *= -1
            self.state.yaw = math.degrees(math.atan2(heading[1], heading[0]))
            #sanity check
            #if ( self.state.x < self.last_x) :
            #    diff = self.state.x - self.last_x
            #    print ('ERROR: X went backwards. Diff: {:.2}'.format(diff))
            #self.last_x = self.state.x

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
                    #TODO: interpolate taking the difference between closest node time and sim time
                    if self.sim_state is ActorSimState.INACTIVE:
                        log.warn("vid {} is now ACTIVE".format(self.vid))
                        self.sim_state = ActorSimState.ACTIVE
                        if self.ghost_mode:
                            self.sim_state = ActorSimState.INVISIBLE
                            log.warn("vid {} is now INVISIBLE".format(self.vid))
                            #workaround for evaluation only
                            if -self.vid in self.simt_traffic.vehicles:
                                self.sim_traffic.vehicles[-self.vid].sim_state = ActorSimState.ACTIVE 
                    xacc = yacc = 0
                    self.state.set_X([node.x, node.xvel, xacc])
                    self.state.set_Y([node.y, node.yvel, yacc])
                    break
            #After trajectory, stay in last position get removed
            if sim_time > end_time:
                if not self.keep_active:
                    self.state.set_X([-9999, 0, 0])
                    self.state.set_Y([-9999,0,0])
                    if self.sim_state is ActorSimState.ACTIVE:
                        self.remove()
                else:
                    self.force_stop()
                    
                
            
        

    
