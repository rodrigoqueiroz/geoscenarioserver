#!/usr/bin/env python
#rqueiroz@uwaterloo.ca
#d43sharm@uwaterloo.ca
# --------------------------------------------
# VehicleState and Motion Plan
# --------------------------------------------
from dataclasses import dataclass
from enum import IntEnum
import numpy as np
import glog as log

class Actor(object):
    def __init__(self, id, name='', start_state=[0.0,0.0,0.0, 0.0,0.0,0.0], frenet_state=[0.0,0.0,0.0, 0.0,0.0,0.0], state=None):
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
        self.state = state or ActorState()
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

    def future_state(self, t):
        """ Predicts a new state based on time and vel.
            Used for collision prediction and charts
            TODO: predict using history
        """
        state = [
            self.state.s + (self.state.s_vel * t) + (self.state.s_acc * t * t),
            self.state.s_vel + (self.state.s_acc * t),
            self.state.s_acc,
            self.state.d + (self.state.d_vel * t) + (self.state.d_acc * t * t),
            self.state.d_vel + (self.state.d_acc * t),
            self.state.d_acc
        ]
        return state


    def stop(self):
        pass

    def force_stop(self):
        #keep current position while forcing vel and acc to 0.0
        self.state.x_vel = self.state.x_acc = self.state.y_vel  = self.state.y_acc = 0.0
        self.state.s_vel = self.state.s_acc = self.state.d_vel  = self.state.d_acc = 0.0
        pass    

    def remove(self):
        self.sim_state = ActorSimState.INACTIVE
        log.warn("Actor id {} is now INACTIVE".format(self.id))

    def tick(self, tick_count, delta_time, sim_time):
        pass

    def follow_trajectory(self,sim_time, trajectory):
        if trajectory:
            start_time = float(trajectory[0].time) #first node
            end_time = float(trajectory[-1].time) #last node
            traj_time = end_time - start_time
            #During trajectory
            if start_time <= sim_time <= end_time:
                #for node in self.trajectory:
                for i in range(len(trajectory)):
                    node = trajectory[i]
                    if node.time < sim_time:
                        continue
                    if self.sim_state is ActorSimState.INACTIVE:
                        log.warn("Actor ID {} is now ACTIVE".format(self.id))
                        self.sim_state = ActorSimState.ACTIVE
                    node.x_vel = node.y_vel = node.x_acc = node.y_acc = 0.0
                    self.state.set_X([node.x, node.x_vel, node.x_acc])
                    self.state.set_Y([node.y, node.y_vel, node.y_acc])
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



@dataclass
class TrajNode:
    x:float = 0.0
    y:float = 0.0
    time:float = 0.0
    speed:float = 0.0
    yaw:float = 0.0

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

@dataclass
class MotionPlan:
    # VECTORSIZE = 15
    s_coef:tuple = tuple([0.0,0.0,0.0,0.0,0.0,0.0]) #6 coef
    d_coef:tuple = tuple([0.0,0.0,0.0,0.0,0.0,0.0]) #6 coef
    t:float = 0                                     #duration
    t_start:float = 0                               #start time [s]
    new_frenet_frame = False
    reversing = False
    VECTORSIZE = len(s_coef) + len(d_coef) + 1 + 1 + 1 + 1
    #Boundaries in Frenet Frame
    #s_start:tuple = [0.0,0.0,0.0]     #start state in s
    #d_start:tuple = [0.0,0.0,0.0]     #start state in d
    #s_target:tuple = [0.0,0.0,0.0]    #target state in s
    #d_target:tuple = [0.0,0.0,0.0]    #target state in d
    #todo: separate trajectory on its own data class

    def get_trajectory(self):
        return ([self.s_coef, self.d_coef,self.t])

    def set_trajectory(self, s_coef, d_coef, t):
        assert(len(self.s_coef) == len(s_coef))
        assert(len(self.d_coef) == len(d_coef))
        self.s_coef = s_coef
        self.d_coef = d_coef
        self.t = t

    #For easy shared memory parsing
    def set_plan_vector(self,P):
        assert(len(P) == MotionPlan.VECTORSIZE)
        ns = len(self.s_coef)
        nd = len(self.d_coef)
        self.s_coef = P[0:ns]
        self.d_coef = P[ns:ns + nd]
        self.t = P[ns + nd]
        self.t_start = P[ns + nd + 1]
        self.new_frenet_frame = P[ns + nd + 2]
        self.reversing = P[ns + nd + 3]

    def get_plan_vector(self):
        # print(self.s_coef)
        assert(len(self.s_coef) == 6)
        assert(len(self.d_coef) == 6)
        return np.concatenate([
            self.s_coef,
            self.d_coef,
            np.asarray([self.t]),
            np.asarray([self.t_start]),
            np.asarray([self.new_frenet_frame]),
            np.asarray([self.reversing])
        ])

