#!/usr/bin/env python
#rqueiroz@uwaterloo.ca
#d43sharm@uwaterloo.ca
# --------------------------------------------
# VehicleState and Motion Plan
# --------------------------------------------
from dataclasses import dataclass
import numpy as np


@dataclass
class VehicleState:
    VECTORSIZE = 9
    FRENET_VECTOR_SIZE = 6
    #sim frame
    x:float = 0.0
    y:float = 0.0
    z:float = 0.0
    #vel
    x_vel:float = 0.0
    y_vel:float = 0.0
    #acc
    x_acc:float = 0.0
    y_acc:float = 0.0
    #orientation
    yaw:float = 0.0
    steer:float = 0.0

    s:float = 0.0
    d:float = 0.0
    #vel
    s_vel:float = 0.0
    d_vel:float = 0.0
    #acc
    s_acc:float = 0.0
    d_acc:float = 0.0

    #For easy shared memory parsing
    def get_state_vector(self):
        arr =  [self.x, self.y,  self.z,
                self.x_vel, self.y_vel,
                self.x_acc, self.y_acc,
                self.yaw,
                self.steer
                ]
        return arr

    def get_frenet_state_vector(self):
        return [
            self.s, self.d,
            self.s_vel, self.d_vel,
            self.s_acc, self.d_acc
        ]

    #For easy shared memory parsing
    def set_state_vector(self, arr):
        self.x, self.y,  self.z, self.x_vel, self.y_vel,self.x_acc, self.y_acc, self.yaw, self.steer,\
            self.s, self.d, self.s_vel, self.d_vel, self.s_acc, self.d_acc = arr

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
