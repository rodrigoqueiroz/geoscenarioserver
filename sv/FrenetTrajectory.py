
#!/usr/bin/env python3
#rqueiroz@uwaterloo.ca
#d43sharm@uwaterloo.ca
# --------------------------------------------
# SIMULATED VEHICLES
# --------------------------------------------

from dataclasses import dataclass
from util.Utils import *

@dataclass
class FrenetTrajectory:
    start_state:tuple = tuple([0.0,0.0,0.0,0.0,0.0,0.0])    # s and d
    target_state:tuple = tuple([0.0,0.0,0.0,0.0,0.0,0.0])   # s and d
    s_coef:tuple = tuple([0.0,0.0,0.0,0.0,0.0,0.0])         #6 coef
    d_coef:tuple = tuple([0.0,0.0,0.0,0.0,0.0,0.0])         #6 coef
    T:float = 0                                             #duration
    consumed_time:float = 0
    projected_trajectory = None

    #Functions
    fs = None
    fs_vel = None
    fs_acc = None
    fs_jerk = None
    fd = None
    fd_vel = None
    fd_acc = None
    fd_jerk = None

    #stats
    feasible:bool = False
    unfeasibility_cause:str = ''
    max_lat_jerk:float=0.0,
    max_long_jerk:float=0.0
    max_long_acc:float=0.0
    max_lat_acc:float=0.0
    collision:float=0.0
    off_lane:float=0.0
    direction:float=0.0

    #costs
    _total_cost:float=9999.0
    time_cost:float=0.0
    effic_cost:float=0.0
    lane_offset_cost:float=0.0
    total_long_jerk_cost:float=0.0
    total_lat_jerk_cost:float=0.0
    total_long_acc_cost:float=0.0
    total_lat_acc_cost:float=0.0
    proximity_cost:float=0.0
    progress_cost:float=0.0

    def array_format(self):
        return [np.array(self.s_coef), np.array(self.d_coef), self.T]
    
    def set_trajectory(self,s_coef, d_coef, duration, projection_precision = None):
        '''
        Set a new trajectory, computes all derivatives, 
        and stores functions for easy access 
        to projected vel, acc, and jerk over time
        '''
        self.s_coef = s_coef
        self.d_coef = d_coef
        self.T = duration
        #derivatives
        s_vel_coef = differentiate(s_coef)
        s_acc_coef = differentiate(s_vel_coef)
        s_jerk_coef = differentiate(s_acc_coef)
        d_vel_coef = differentiate(d_coef)
        d_acc_coef = differentiate(d_vel_coef)
        d_jerk_coef = differentiate(d_acc_coef)
        #store equations
        self.fs = to_equation(s_coef)
        self.fs_vel = to_equation(s_vel_coef)
        self.fs_acc = to_equation(s_acc_coef)
        self.fs_jerk = to_equation(s_jerk_coef)
        self.fd = to_equation(d_coef)
        self.fd_vel = to_equation(d_vel_coef)
        self.fd_acc = to_equation(d_acc_coef)
        self.fd_jerk = to_equation(d_jerk_coef)
        #projects
        if projection_precision is not None:
            self.projected_trajectory = self.project(projection_precision)
        else:
            self.projected_trajectory = None

    def get_state_at(self, time):
        '''
        Returns a predicted frenet state for a given time in the trajectory
        '''
        #limit max time
        if time > self.T: 
            time = self.T
        return [self.fs(time), self.fs_vel(time), self.fs_acc(time),
                self.fd(time), self.fd_vel(time), self.fd_acc(time)]

    def project(self,precision = 10):
        '''Projects trajectory from start to goal and split in equally distributed parts.
            Higher precision returns more measures, but impacts performance.
        '''
        projected = []
        dt = float(self.T) / precision
        for i in range(precision):
            #t = float(i) / split * self.T
            t = dt * i
            state_s = [self.fs(t), self.fs_vel(t), self.fs_acc(t), self.fs_jerk(t)]
            state_d = [self.fd(t), self.fd_vel(t), self.fd_acc(t), self.fd_jerk(t)]
            projected.append([t,state_s,state_d])
        return projected

    def get_total_cost(self):
        self._total_cost = sum([self.time_cost,
                            self.effic_cost,
                            self.lane_offset_cost,
                            self.total_long_jerk_cost,
                            self.total_lat_jerk_cost,
                            self.total_long_acc_cost,
                            self.total_lat_acc_cost,
                            self.proximity_cost,
                            self.progress_cost])
        return self._total_cost

@dataclass
class MotionPlan:
    trajectory:FrenetTrajectory = FrenetTrajectory()
    start_time:float = 0            #sim start time [s]
    new_frenet_frame = False        #if a new frenet frame was generated
    reversing = False               #if the plan is reversed
    tick_count = None
    ref_path_origin:float = None
    
    #For easy shared memory parsing
    def get_vector_length(self):
        return len(self.trajectory.s_coef) + len(self.trajectory.d_coef) + 6

    def set_plan_vector(self,P):
        assert(len(P) == self.get_vector_length())
        ns = len(self.trajectory.s_coef)
        nd = len(self.trajectory.d_coef)
        s_coef = P[0:ns]
        d_coef = P[ns:ns + nd]
        t = P[ns + nd]
        self.start_time = P[ns + nd + 1]
        self.new_frenet_frame = P[ns + nd + 2]
        self.reversing = P[ns + nd + 3]
        self.tick_count = P[ns + nd + 4]
        self.ref_path_origin = P[ns + nd + 5]
        self.trajectory.set_trajectory(s_coef,d_coef,t)

    def get_plan_vector(self):
        assert(len(self.trajectory.s_coef) == 6)
        assert(len(self.trajectory.d_coef) == 6)
        return np.concatenate([
            self.trajectory.s_coef,
            self.trajectory.d_coef,
            np.asarray([self.trajectory.T]),
            np.asarray([self.start_time]),
            np.asarray([self.new_frenet_frame]),
            np.asarray([self.reversing]),
            np.asarray([self.tick_count]),
            np.asarray([self.ref_path_origin])
        ])
