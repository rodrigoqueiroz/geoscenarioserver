#!/usr/bin/env python3
#rqueiroz@uwaterloo.ca
#d43sharm@uwaterloo.ca
# --------------------------------------------
# VehicleState and Motion Plan
# --------------------------------------------
from dataclasses import dataclass
from enum import IntEnum
## sean: to_equation and differentiate are deprecated
# from util.Utils import to_equation, differentiate
from SimConfig import *
import numpy as np
import glog as log

class Actor(object):
    def __init__(self, id, name='', start_state=[0.0,0.0,0.0, 0.0,0.0,0.0], frenet_state=[0.0,0.0,0.0, 0.0,0.0,0.0], yaw=0.0, state=None):
        self.id = id
        self.name = name
        self.sim_state = ActorSimState.ACTIVE
        self.radius = 0.0
        self.type = None
        self.model = None
        self.ghost_mode = False
        self.sim_traffic = None

        #state
        #start state in sim frame
        self.state:ActorState = state or ActorState()
        self.state.x = start_state[0]
        self.state.x_vel = start_state[1]
        self.state.x_acc = start_state[2]
        self.state.y = start_state[3]
        self.state.y_vel = start_state[4]
        self.state.y_acc = start_state[5]
        # start state in frenet
        self.state.s = frenet_state[0]
        self.state.s_vel = frenet_state[1]
        self.state.s_acc = frenet_state[2]
        self.state.d = frenet_state[3]
        self.state.d_vel = frenet_state[4]
        self.state.d_acc = frenet_state[5]

        self.state.yaw = yaw

    def future_state(self, dt):
        """ Predicts a new state based on time and vel.
            Used for collision prediction and charts
            Note: Acc can rapidly change. Using the current acc to predict future
            can lead to overshooting forward or backwards when vehicle is breaking
            TODO: predict using history + kalman filter
        """
        state = [
            self.state.s + (self.state.s_vel * dt), #+ (self.state.s_acc * dt * dt),
            self.state.s_vel, #+ (self.state.s_acc * dt),
            self.state.s_acc,
            self.state.d + (self.state.d_vel * dt), #+ (self.state.d_acc * dt * dt),
            self.state.d_vel, #+ (self.state.d_acc * dt),
            self.state.d_acc
        ]
        return state


    def stop(self):
        pass

    def force_stop(self):
        #keep current position while forcing vel and acc to 0.0
        self.state.x_vel = self.state.x_acc = self.state.y_vel = self.state.y_acc = 0.0
        self.state.s_vel = self.state.s_acc = self.state.d_vel = self.state.d_acc = 0.0
        pass

    def remove(self):
        self.sim_state = ActorSimState.INACTIVE
        log.warn("Actor id {} is now INACTIVE".format(self.id))

    def tick(self, tick_count, delta_time, sim_time):
        pass

    def follow_trajectory(self, sim_time, trajectory):
        if trajectory:
            start_time = float(trajectory[0].time) #first node
            end_time = float(trajectory[-1].time) #last node
            #During trajectory
            if start_time <= sim_time <= end_time:
                #Trajectory starts
                if self.sim_state is ActorSimState.INACTIVE:
                    log.warn("Actor ID {} is now ACTIVE".format(self.id))
                    self.sim_state = ActorSimState.ACTIVE
                    if self.ghost_mode:
                        self.sim_state = ActorSimState.INVISIBLE
                        log.warn("vid {} is now INVISIBLE".format(self.id))
                    if EVALUATION_MODE:
                        if -self.id in self.sim_traffic.vehicles:
                            self.sim_traffic.vehicles[-self.id].sim_state = ActorSimState.ACTIVE
                            log.warn("vid {} is now ACTIVE".format(-self.id))

                #find closest pair of nodes
                for i in range(len(trajectory)-1):
                    n1 = trajectory[i]
                    n2 = trajectory[i+1]
                    if (n1.time <= sim_time < n2.time): # n2 must be strictly after n1
                        dx = n2.x - n1.x
                        dx_vel = n2.x_vel - n1.x_vel
                        dy = n2.y - n1.y
                        dy_vel = n2.y_vel - n1.y_vel
                        dt = n2.time - n1.time
                        x_acc = dx_vel/dt
                        y_acc = dy_vel/dt
                        pdiff = (sim_time - n1.time)/dt #always positive
                        # Interpolate
                        self.state.set_X([n1.x + (dx * pdiff), n2.x_vel, x_acc])
                        self.state.set_Y([n1.y + (dy * pdiff), n2.y_vel, y_acc])
                        self.state.yaw = n1.yaw
                        break

            #After trajectory, stay in last position or get removed
            if sim_time > end_time:
                if not self.keep_active:
                    self.state.set_X([-9999, 0, 0])
                    self.state.set_Y([-9999, 0, 0])
                    if self.sim_state is ActorSimState.ACTIVE:
                        self.remove()
                else:
                    self.force_stop()

    def follow_path(self, delta_time, sim_time, path):
        if path:
            # Which path node have we most recently passed
            node_checkpoint = 0

            # Ideally we should first calculate acceleration, then velocity, then position (euler integration)
            # For now, we'll ignore acceleration
            # TODO: This could be improved by saving the current path node instead of having to find it again every tick

            # Calculate velocity
            for i in range(len(path)-1):
                n1 = path[i]
                n2 = path[i+1]
                if (n1.s <= self.state.s <= n2.s):
                    # For now we assume that the velocity is specified at each path point or none of them
                    # Later we could instead interpolate between points with speed specified
                    if n1.speed is not None and n2.speed is not None:
                        # Interpolate the velocity
                        ratio = (self.state.s - n1.s)/(n2.s - n1.s)
                        self.state.s_vel = n1.speed + (n2.speed - n1.speed) * ratio

                    node_checkpoint = i
                    break

            # Calculate frenet position
            self.state.s += (self.state.s_vel * delta_time)

            # Now calculate the cartesian state from the frenet state
            for i in range(node_checkpoint, len(path)-1):
                n1 = path[i]
                n2 = path[i+1]
                if (n1.s <= self.state.s <= n2.s):
                    dx = n2.x - n1.x
                    dy = n2.y - n1.y
                    d = n2.s - n1.s
                    # Calculate position
                    ratio = (self.state.s - n1.s)/(d)
                    self.state.x = n1.x + (dx) * ratio
                    self.state.y = n1.y + (dy) * ratio

                    # Calculate velocity vector
                    self.state.x_vel = self.state.s_vel * dx/d
                    self.state.y_vel = self.state.s_vel * dy/d

                    # Calculate yaw
                    self.state.yaw = math.degrees(math.atan2(dy, dx))
                    break

            # Reached the end of the path
            if self.state.s > path[-1].s:
                self.force_stop()
                if not self.keep_active:
                    self.state.set_X([-9999, 0, 0])
                    self.state.set_Y([-9999,0,0])
                    if self.sim_state is ActorSimState.ACTIVE:
                        self.remove()

@dataclass
class TrajNode:
    x:float = 0.0
    y:float = 0.0
    x_vel:float = None  # calculated as dx/dt, the first node same as the second
    y_vel:float = None  # calculated as dy/dt, the first node same as the second
    time:float = 0.0
    yaw:float = 0.0

@dataclass
class PathNode:
    x:float = 0.0      # [m]
    y:float = 0.0      # [m]
    s:float = 0.0      # [m]
    speed:float = 0.0  # [m/s]

@dataclass
class ActorSimState(IntEnum):
    INACTIVE = 0          #not in simulation, not present in traffic, and not visible for other agents
    ACTIVE = 1            #in simulation and visible to other agents
    INVISIBLE = 2         #in simulation but NOT visible to other agents (for reference)

@dataclass
class ActorState:
    #sim frame
    x:float = 0.0
    x_vel:float = 0.0
    x_acc:float = 0.0

    y:float = 0.0
    y_vel:float = 0.0
    y_acc:float = 0.0

    z:float = 0.0
    z_vel:float = 0.0
    z_acc:float = 0.0

    #frenet state
    s:float = 0.0
    s_vel:float = 0.0
    s_acc:float = 0.0
    d:float = 0.0
    d_vel:float = 0.0
    d_acc:float = 0.0

    #the direction the actor is facing
    yaw:float = 0.0

    #For easy shared memory parsing
    def get_cart_state_vector(self):
        return [self.x, self.x_vel, self.x_acc,
                self.y,  self.y_vel, self.y_acc]

    def get_frenet_state_vector(self):
        return [self.s, self.s_vel, self.s_acc,
                self.d, self.d_vel, self.d_acc]

    def set_state_vector(self, arr):
        self.set_X(arr[0:3])
        self.set_Y(arr[3:6])
        self.set_S(arr[6:9])
        self.set_D(arr[9:12])
        self.yaw = arr[12]

    #For easy shared memory parsing
    def get_state_vector(self):
        combined = self.get_cart_state_vector() + self.get_frenet_state_vector() + [self.yaw]
        return combined

    def get_X(self):
        return [self.x,self.x_vel,self.x_acc]

    def get_Y(self):
        return [self.y,self.y_vel,self.y_acc]

    def set_X(self, X):
        self.x, self.x_vel, self.x_acc = X

    def set_Y(self, Y):
        self.y, self.y_vel, self.y_acc = Y

    def get_S(self):
        return [self.s, self.s_vel, self.s_acc]

    def set_S(self, S):
        self.s, self.s_vel, self.s_acc = S

    def get_D(self):
        return [self.d, self.d_vel, self.d_acc]

    def set_D(self, D):
        self.d, self.d_vel, self.d_acc = D

    def get_cartesian_speed(self):
        return math.sqrt(abs(self.x_vel)**2 + abs(self.y_vel)**2)

@dataclass
class VehicleState(ActorState):
   steer:float = 0.0

@dataclass
class PedestrianState(ActorState):
   pass

@dataclass
class StaticObject():
    id:int = 0
    x:float = 0.0
    y:float = 0.0
    s:float = 0.0
    d:float = 0.0
    model:str = ''


