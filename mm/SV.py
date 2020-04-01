#!/usr/bin/env python
#rqueiroz@gsd.uwaterloo.ca
# --------------------------------------------
# SIMULATED HUMAN VEHICLE AND FRENET PATH
# --------------------------------------------

from matplotlib import pyplot as plt
from Utils import *
from SVPlanner import *
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
M_VELKEEPING = 0
M_LANECHANGE = 1
M_CUTIN = 2
M_FOLLOW = 3
M_STOP = 4

# A Simulated Vehicle
class SV(object):
    
    def __init__(self, id, start_state):
        #id
        self.id = id
        #vehicle stats
        self.radius = 1.0

        #Mission in sim frame
        self.xgoal = 0
        self.ygoal = 0

        #Planning
        lookahead_dist = 0
        self.trajectory = None #[[0,0,0,0,0,0],[0,0,0,0,0,0],[0]]
        self.cand_trajectories = None #plotting only

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

        # TODO: init with ids?
        self.sm = SVSharedMemory(self.id)
       
    
    def setbehavior(self, btree, target_id = None, goal_pos = None):
        """ btree: is the root behavior tree. By default, the tree is velocity keeping.
            target: can be used in many subtrees to force a maneuver towards one specific vehicle. 
                    Usually, Ego is the target. Only one target is allowed.
                    Without a defined target, maneuvers will adapt to vehicles around them as target.
        """
        self.btree = btree
        self.target_id = target_id

    def tick(self,time,vehicles):
        if (self.trajectory):
            self.execute(time)

            # TODO: some transformation out of frenet frame
            # trajectory is (s_coefs, d_coefs, t)
            self.x = to_equation(self.trajectory[0])(self.trajectory[2])
            self.y = to_equation(self.trajectory[1])(self.trajectory[2])

            print([self.sm.shm_key, self.x, self.y])
            # Write the position and yaw to shared memory
            self.sm.write([self.x, self.y, self.z], self.yaw)

        print('plan')
        self.plan(vehicles)
        

    # TODO: this is being slow?
    def plan(self,vehicles):
        
        #Road Config
        #todo: Get road attributes from laneletmap. Hardcoding now.
        lane_conf = LaneConfig(30,4,0)

        #Behaviour layer
        replan = False
        if (self.btree==BT_VELKEEPING):
            maneuver = M_VELKEEPING
            mlk_config = MVelKeepingConfig((MIN_VELOCITY, MAX_VELOCITY), (VK_MIN_TIME,VK_MAX_TIME))
            replan = True
        elif (self.btree==BT_FOLLOW):
            maneuver = M_FOLLOW
            mfo_config = MFollowConfig( (FL_MIN_TIME, FL_MAX_TIME), 4, 40)
            replan = True
        elif (self.btree==BT_STOP):
            maneuver = M_STOP
            mst_config = MStopConfig(time_range = (VK_MIN_TIME,VK_MAX_TIME))
            replan = True
        
        # self.btree==BT_PARKED

        #Micro maneuver layer
        if (replan):
            start_state = [self.s_pos, self.s_vel, self.s_acc, self.d_pos, self.d_vel, self.d_acc]
            if (maneuver==M_STOP):
                candidates, best = plan_stop(start_state, mst_config, lane_conf, 200, vehicles, None) 
            elif (maneuver==M_VELKEEPING):
                candidates, best = plan_velocity_keeping(start_state, mlk_config, lane_conf, vehicles, None) 
            elif (maneuver==M_FOLLOW):
                candidates, best = plan_following(start_state, mfo_config, lane_conf, self.target_id, vehicles)

            #if (maneuver==LANECHNAGE):
            #    best = ST_LaneChange(hv.start_state, goal_state, T) #returns tuple (s[], d[], t)
            #if (maneuver==CUTIN):
            #    best = ST_CutIn( hv.start_state, delta, T,vehicles,target_id) #, True, True) #returns tuple (s[], d[], t)
            #    candidates, best  = OT_CutIn( hv.start_state, delta, T, vehicles,target_id,True,True) #returns tuple (s[], d[], t)
            
            if (best):
                self.trajectory = best
            if (candidates):
                self.cand_trajectories = candidates
        
        
    # called every tick, before planning
    def execute(self,time_step):
        #Consume trajectory at given time
        if (self.trajectory):
            s_eq = to_equation(self.trajectory[0]) 
            d_eq = to_equation(self.trajectory[1]) 
            new_s = s_eq(time_step)
            new_d = d_eq(time_step)
            self.s_pos = new_s
            self.d_pos = new_d

            s_vel_coef = differentiate(self.trajectory[0])
            s_vel_eq = to_equation(s_vel_coef)
            self.s_vel = s_vel_eq(time_step)

            #self.d_vel = new_d[1]
            #self.d_acc = new_d[2]
            #hv.start_state[0] = new_s
            #hv.start_state[3] = new_d

    def plot(self):
        gca = plt.gca()
        plt.plot( self.s_pos, self.d_pos, "v")
        circle1 = plt.Circle((self.s_pos, self.d_pos), self.radius, color='b', fill=False)
        gca.add_artist(circle1)
        #label = "id {} | state[{}m, {}m/s, {}m/ss] ".format(self.id,self.s_pos, self.s_vel,self.s_acc)
        label = "id{}| [ {:.3} , {:.3} , {:.3} ] ".format(self.id, self.s_pos, self.s_vel, self.s_acc)


        gca.text(self.s_pos, self.d_pos+1.5, label, style='italic')

        #if (self.cand_trajectories):
            #plot_multi_trajectory(self.cand_trajectories,self.trajectory, None, False, False)
        
        if (self.trajectory):        
            plot_trajectory(self.trajectory[0], self.trajectory[1], self.trajectory[2],'blue')
            #plot_single_trajectory(self.trajectory, None, False, True)

        

        

      
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
