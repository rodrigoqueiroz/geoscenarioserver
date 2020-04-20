#!/usr/bin/env python
#rqueiroz@gsd.uwaterloo.ca
# --------------------------------------------
# SIMULATED HUMAN CONTROLLED VEHICLE AND FRENET PATH
# --------------------------------------------

from matplotlib import pyplot as plt
from Utils import *
from SVPlanner import *
from TickSync import TickSync
from shared_mem.SVSharedMemory import *
import math


#BTree #todo: pytrees
BT_PARKED = 0 #default, car is stopped
BT_DRIVE = 1  #follow a route with normal driving. Can switch to follow, or stop
BT_STOP = 2
BT_VELKEEPING = 3
BT_FOLLOW = 4 #follow a specific target
BT_CUTIN = 5  #reach and cut in a specific target


#maneuvers
M_VELKEEPING = 1
M_LANECHANGE = 2
M_CUTIN = 3
M_FOLLOW = 4
M_STOP = 5

# A Simulated Vehicle
class SV(object):
    
    def __init__(self, id, start_state):
        #id
        self.id = id
       
        #Mission in sim frame
        self.xgoal = 0
        self.ygoal = 0

        #Planning
        lookahead_dist = 0
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
        self.x = 0
        self.y = 0
        self.z = 0
        self.yaw = 0
        self.vel = 0
        self.acc = 0
        #state in moving frenet frame
        self.s_pos = start_state[0]
        self.s_vel = start_state[1]
        self.s_acc = start_state[2]
        self.d_pos = start_state[3]
        self.d_vel = start_state[4]
        self.d_acc = start_state[5]

        #todo: save the path in vector with time stamps to avoid multiple function calculations?
        self.global_path = []
        self.local_path = []
        self.frenet_path = []
        #todo: change start and goal state to sim frame
        
        self.sync_bt = TickSync(rate=0.5, block=False, verbose=True, label="BT")
        self.sync_mm = TickSync(rate=0.5, block=False, verbose=True, label="MM")
        
        # Id's are used to distinguish between shared memory for different vehicles
        # In the future we might want to use one shared memory for all vehicles
        # to make it easier to manage.
        self.sm = SVSharedMemory(self.id)
    
    
    def get_stats(self):
        vehicle_state =  [self.s_pos, self.s_vel, self.s_acc, self.d_pos, self.d_vel, self.d_acc]
        return self.id, vehicle_state, self.trajectory, self.cand_trajectories
    
    
    def setbehavior(self, btree, target_id = None, goal_pos = None):
        """ btree: is the root behavior tree. By default, the tree is velocity keeping.
            target: can be used in many subtrees to force a maneuver towards one specific vehicle. 
                    Usually, Ego is the target. Only one target is allowed.
                    Without a defined target, maneuvers will adapt to vehicles around them as target.
        """
        self.btree = btree
        self.target_id = target_id

    def tick(self,delta_time,vehicles):
        #Execution TICK (every tick)
        if (self.trajectory):
            self.trajectory_time += delta_time
            self.compute_new_pose(self.trajectory_time)

        #Road Config
        #todo: Get road attributes from laneletmap. Hardcoding now.
        lane_conf = LaneConfig(30,4,0)
   
        #BT Tick
        if self.sync_bt.tick():
            maneuver,m_config = self.plan_behavior(lane_conf, vehicles)
        
        #Motion Planning Tick
        if self.sync_mm.tick():
            if (maneuver):
                self.plan_maneuver(maneuver,m_config,lane_conf,vehicles)
        
        #Other Actions (e.g., turn signal)        


    #Consume trajectory based on a given time and update pose
    #Optimized with pre computed derivatives and equations
    def compute_new_pose(self,time):
        self.s_pos = self.s_eq(time)
        self.s_vel = self.s_vel_eq(time)
        self.s_acc = self.s_acc_eq(time)
        self.d_pos = self.d_eq(time)
        self.d_vel = self.d_vel_eq(time)
        self.d_acc = self.d_acc_eq(time)

        # TODO: some transformation out of frenet frame
        # trajectory is (s_coefs, d_coefs, t)
        #self.x = to_equation(self.trajectory[0])(self.trajectory[2])
        #self.y = to_equation(self.trajectory[1])(self.trajectory[2])
        self.x = self.s_pos * 10
        self.y = self.d_pos * 10

        #print([self.x, self.y])      
        
        #Update ShM Pose
        # Write the position and yaw to shared memory
        self.sm.write([self.x, self.y, self.z], self.yaw)

    def plan_behavior(self,lane_conf,vehicles):
        #TODO: add BTrees and return either a Maneuver or Action
        maneuver = None
        if (self.btree==BT_VELKEEPING):
            maneuver = M_VELKEEPING
            m_config = MVelKeepingConfig((MIN_VELOCITY, MAX_VELOCITY), (VK_MIN_TIME,VK_MAX_TIME))
        elif (self.btree==BT_FOLLOW):
            maneuver = M_FOLLOW
            m_config = MFollowConfig( (FL_MIN_TIME, FL_MAX_TIME), 4, 40)
        elif (self.btree==BT_STOP):
            maneuver = M_STOP
            m_config = MStopConfig(time_range = (VK_MIN_TIME,VK_MAX_TIME))
        #self.btree==BT_PARKED
        return maneuver, m_config

    def plan_maneuver(self,maneuver,m_config,lane_conf,vehicles):
        vehicle_state =  [self.s_pos, self.s_vel, self.s_acc, self.d_pos, self.d_vel, self.d_acc]
        print('Plan Maneuver, State {}'.format(vehicle_state[0],vehicle_state[1],vehicle_state[2],vehicle_state[3],vehicle_state[4],vehicle_state[5]))
        #Micro maneuver layer
        start_state = [self.s_pos, self.s_vel, self.s_acc, self.d_pos, self.d_vel, self.d_acc]
        if (maneuver==M_STOP):
            candidates, best = plan_stop(start_state, m_config, lane_conf, 200, vehicles, None) 
        elif (maneuver==M_VELKEEPING):
            candidates, best = plan_velocity_keeping(start_state, m_config, lane_conf, vehicles, None) 
        elif (maneuver==M_FOLLOW):
            candidates, best = plan_following(start_state, m_config, lane_conf, self.target_id, vehicles)
        #if (maneuver==LANECHNAGE):
        #    best = ST_LaneChange(hv.start_state, goal_state, T) #returns tuple (s[], d[], t)
        #if (maneuver==CUTIN):
        #    best = ST_CutIn( hv.start_state, delta, T,vehicles,target_id) #, True, True) #returns tuple (s[], d[], t)
        #    candidates, best  = OT_CutIn( hv.start_state, delta, T, vehicles,target_id,True,True) #returns tuple (s[], d[], t)
        

        
        if (best):
            self.set_new_trajectory(best,candidates)

    def set_new_trajectory(self,trajectory, candidates):
            self.trajectory = trajectory
            self.trajectory_time = 0 
            #Pre Computer derivatives and Equations
            #S
            s_coef = trajectory[0]
            self.s_eq = to_equation(s_coef) 
            s_vel_coef = differentiate(s_coef)
            self.s_vel_eq = to_equation(s_vel_coef)
            s_acc_coef = differentiate(s_vel_coef)
            self.s_acc_eq = to_equation(s_acc_coef)
            #D
            d_coef = trajectory[1]
            self.d_eq = to_equation(d_coef) 
            d_vel_coef = differentiate(d_coef)
            self.d_vel_eq = to_equation(d_vel_coef)
            d_acc_coef = differentiate(d_vel_coef)
            self.d_acc_eq = to_equation(d_acc_coef)
      
    def state_in(self, t):
        if (self.trajectory):
            #s
            s_coef = self.trajectory[0]
            svel_coef = differentiate(s_coef)
            sacc_coef = differentiate(svel_coef)
            s_eq = to_equation(s_coef)
            svel_eq = to_equation(svel_coef)
            sacc_eq = to_equation(sacc_coef)
            #d
            d_coef = self.trajectory[1]
            dvel_coef = differentiate(d_coef)
            dacc_coef = differentiate(dvel_coef)
            d_eq = to_equation(d_coef)
            dvel_eq = to_equation(dvel_coef)
            dacc_eq = to_equation(dacc_coef)
        
            predicted_state = [
                s_eq(t),
                svel_eq(t),
                sacc_eq(t),
                d_eq(t),
                dvel_eq(t),
                dacc_eq(t),
            ]
            return predicted_state
        else: #No trejectory. Predicted state is current state
            return [self.s_pos, self.s_vel, self.s_acc, self.d_pos, self.d_vel, self.d_acc] 
    
 

#todo: change
def calc_global_paths(fplist, csp):

    for fp in fplist:

        # calc global positions
        for i in range(len(fp.s)):
            ix, iy = csp.calc_position(fp.s[i])
            if ix is None:
                break
            iyaw = csp.calc_yaw(fp.s[i])
            di = fp.d[i]
            fx = ix + di * math.cos(iyaw + math.pi / 2.0)
            fy = iy + di * math.sin(iyaw + math.pi / 2.0)
            fp.x.append(fx)
            fp.y.append(fy)

        # calc yaw and ds
        for i in range(len(fp.x) - 1):
            dx = fp.x[i + 1] - fp.x[i]
            dy = fp.y[i + 1] - fp.y[i]
            fp.yaw.append(math.atan2(dy, dx))
            fp.ds.append(math.hypot(dx, dy))

        fp.yaw.append(fp.yaw[-1])
        fp.ds.append(fp.ds[-1])

        # calc curvature
        for i in range(len(fp.yaw) - 1):
            fp.c.append((fp.yaw[i + 1] - fp.yaw[i]) / fp.ds[i])

    return fplist

#todo: change
class Frenet_path:

    def __init__(self):
        self.T = []
        self.D = []
        self.D_VEL = []
        self.D_ACC = []
        self.S = []
        self.S_VEL = []
        self.S_ACC = []
        #cossts
        self.CD = 0.0
        self.CV = 0.0
        self.CF = 0.0

        self.X = []
        self.Y = []
        self.YAW = []
