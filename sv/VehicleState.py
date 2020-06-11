#!/usr/bin/env python
#rqueiroz@gsd.uwaterloo.ca
#d43sharm@uwaterloo.ca
# --------------------------------------------
# VehicleState and Motion Plan
# --------------------------------------------
from dataclasses import dataclass
import numpy as np

# question - is this all in cartesian?
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

    # frenet frame - make another FrenetState class?
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
    VECTORSIZE = 13
    s_coef:tuple = tuple([0.0,0.0,0.0,0.0,0.0])     #5 coef
    d_coef:tuple = tuple([0.0,0.0,0.0,0.0,0.0,0.0]) #6 coef
    t:float = 0                                     #duration
    t_start:float = 0                               #start time [s]
    #Boundaries in Frenet Frame
    #s_start:tuple = [0.0,0.0,0.0]     #start state in s
    #d_start:tuple = [0.0,0.0,0.0]     #start state in d
    #s_target:tuple = [0.0,0.0,0.0]    #target state in s
    #d_target:tuple = [0.0,0.0,0.0]    #target state in d
    #todo: separate trajectory on its own data class

    def get_trajectory(self):
        return ([self.s_coef, self.d_coef,self.t])

    def set_trajectory(self, s_coef, d_coef, t):
        self.s_coef = s_coef 
        self.d_coef = d_coef
        self.t = t
        
    #For easy shared memory parsing
    def set_plan_vector(self,P):
        self.s_coef = P[0:5]
        self.d_coef = P[5:11]
        self.t = P[11]
        self.t_start = P[12]
    
    def get_plan_vector(self):
        return np.concatenate([
            self.s_coef, 
            self.d_coef,
            np.asarray([self.t]),
            np.asarray([self.t_start])
            ])
