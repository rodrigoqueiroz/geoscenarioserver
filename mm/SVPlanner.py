#!/usr/bin/env python
# ---------------------------------------------
# GEOSCENARIO SIMULATION VEHICLE PLANNER
# --------------------------------------------
__author__ = "Rodrigo Queiroz"
__email__ = "rqueiroz@gsd.uwaterloo.ca"

import numpy as np
import random
from CostFunctions import *
from Constants import *
from Plots import *
from Utils import *

#todo: add all config structs and default values (CONSTAMTS) to a separate file
class LaneConfig():
    def __init__(self, max_velocity, left_boundary, right_boundary):
        self.max_velocity = max_velocity
        self.left_boundary = left_boundary
        self.right_boundary = right_boundary

class MStopConfig():
    def __init__(self, time_range, deceleration_range = None, min_distance = 30):
        #self.distance_range = distance_range
        #self.stop_pos = stop_pos
        self.time_range = time_range
        self.deceleration_range = deceleration_range
        self.min_distance = min_distance
        
class MVelKeepingConfig():
    def __init__(self, vel_range, time_range ):
        self.vel_range = vel_range
        self.time_range = time_range

class MFollowConfig():
    def __init__(self, time_range,time_gap, distance):
        self.time_range = time_range
        self.time_gap = time_gap
        self.distance = distance
        self.min_distance  = 50.0 #min distance to collision [meters]
        self.min_ttc = 10.0 #min time to collision [seconds]

class MLaneChangeConfig():
    def __init__(self, time_range, time_gap, distance):
        self.time_range = time_range
        self.time_gap = time_gap
        self.distance = distance

class MCutInConfig():
    def __init__(self, time_range, time_gap, distance, delta):
        self.time_range = time_range
        self.time_gap = time_gap
        self.distance = distance
        self.delta = delta
        




#===MICRO MANEUVERS===s

#Stop
#Stop can be a stop request by time and/or distance from current pos.
#Or optionally have a specific target position to stop (stop line, before an object, etc).
def plan_stop(start_state, man_config, lane_config, target_pos , vehicles = None, obstacles = None ):
    print ('PLAN STOP')
    if (start_state[0] >= target_pos):
        print ('Stop target position is too close: {}'.format(target_pos - start_state[0]))
        return None,None

    
    #start
    s_start = start_state[:3]
    d_start = start_state[3:]
    #time
    min_t = man_config.time_range[0]
    max_t = man_config.time_range[1]
    t_step = PLAN_TIME_STEP    
    #distance
    #min_s = man_config.distance_range[0]
    #max_s = man_config.distance_range[1]
    #s_step = STOP_DISTANCE_STEP    
    
    #road
    min_d = lane_config.right_boundary
    max_d = lane_config.left_boundary
    d_step = ROAD_W_STEP
    #targets
    target_set = []
    #generates alternative targets
    for t in np.arange(min_t, max_t+t_step, t_step):
        #longitudinal movement: goal is to reach vel and acc 0
        #for si in np.arange(min_s, max_s+s_step, s_step):
            #s_pos = s_start[0] + si
            s_target = [target_pos,0,0] #target vel and acc is 0
            #lateral movement
            for di in np.arange(min_d, max_d, d_step):
                d_target = [di,0,0] 
                #add target
                target_set.append((s_target,d_target,t))
            
    #fit jerk optimal trajectory between two points in s and d per goal
    trajectories = []  
    for target in target_set:
        s_target, d_target, t = target
        s_coef = quintic_polynomial_solver(s_start, s_target, t)
        d_coef = quintic_polynomial_solver(d_start, d_target, t)
        trajectories.append(tuple([s_coef, d_coef, t]))

    #evaluate and select "best" trajectory    
    #best = min(trajectories, key=lambda tr: follow_cost(tr, T, vehicles))
    best = min(trajectories, key=lambda tr: stop_cost(tr, vehicles))
   
    #return trajectories and best
    return trajectories, best


#Velocity Keeping
#Driving with no vehicle directly ahead
#No target point, but needs to adapt to a desired velocity
def plan_velocity_keeping(start_state, man_config, lane_config, vehicles = None, obstacles = None):
    print ('Maneuver: Velocity Keeping')

    s_start = start_state[:3]
    d_start = start_state[3:]

    #time
    #T = VK_TIME
    min_t = man_config.time_range[0]
    max_t = man_config.time_range[1]
    t_step = PLAN_TIME_STEP
    #vel
    min_vel = man_config.vel_range[0]
    max_vel = man_config.vel_range[1]
    vel_step = VELOCITY_STEP 
    #road
    min_d = lane_config.right_boundary
    max_d = lane_config.left_boundary
    d_step = ROAD_W_STEP
    
    #
    target_set = []
    
    #generates alternative targets
    for t in np.arange(min_t, max_t, t_step):
        #longitudinal movement: goal is to keep velocity
            for vel in np.arange(min_vel, max_vel, vel_step):
                s_target = [0,0,0]
                s_target[0] = 0     # pos is not relevant. Will solve a quartial polynomial instead
                s_target[1] = vel   # target velocity
                s_target[2] = 0     # acc
                #s_target[0] = s_start[0] + (vel * t) + s_start[2] * t**2 / 2.0   #predicted position
                
                #lateral movement
                for di in np.arange(min_d, max_d, d_step):
                    d_target = [di,0,0] 
                    #add target
                    target_set.append((s_target,d_target,t))
    
    
    #fit jerk optimal trajectory between two points in s and d per goal
    trajectories = []
    for target in target_set:
        s_target, d_target, t = target
        s_coef = quartic_polynomial_solver(s_start, s_target, t)
        d_coef = quintic_polynomial_solver(d_start, d_target, t)
        trajectories.append(tuple([s_coef, d_coef, t]))
        # print(s_target)
        # print(d_target)
        # print(t)

    #evaluate and select "best" trajectory    
    #best = min(trajectories, key=lambda tr: velocity_keeping_cost(tr, T, vehicles))
    best = min(trajectories, key=lambda tr: velocity_keeping_cost(tr, vehicles))
   
    #return trajectories and best
    return trajectories, best

#Vehicle Following
#Moving target point, requiring a certain temporal safety distance 
#to the vehicle ahead (constant time gap law).
#Predict leading vehicle (assume constant acceleration)
def plan_following(start_state, man_config, lane_config, target_v_id, vehicles = None, obstacles = None):
    print ('Maneuver: Follow vehicle')

    s_start = start_state[:3]
    d_start = start_state[3:]
    
    #T = FL_TIME
    min_t = man_config.time_range[0]
    max_t = man_config.time_range[1]
    t_step = PLAN_TIME_STEP
    time_gap = man_config.time_gap
    distance = man_config.distance
    
    #road
    min_d = lane_config.right_boundary
    max_d = lane_config.left_boundary
    d_step = ROAD_W_STEP
    
    target_set = []
    print (vehicles)

    #generates alternative targets
    for t in np.arange(min_t, max_t, t_step):
        #longitudinal movement: goal is to keep safe distance from s lv
        s_lv = vehicles[target_v_id].state_in(t)[:3]
        s_target = [0,0,0]
        s_target[0] = s_lv[0] - (distance + time_gap * s_lv[1])    #pos in s
        s_target[1] = s_lv[1] - (time_gap * s_lv[2])    #vel in s
        s_target[2] = s_lv[1]                           #acc in s #final acc should be 0?
        #if (s_lv[0] < s_target[0] ):
        #    print("does it make sense?")
        #    print(s_lv)
        #    print(s_target)
        #lateral movement
        for di in np.arange(min_d, max_d, d_step):
            d_target = [di,0,0] 
            target_set.append((s_target,d_target,t))
            
    #fit jerk optimal trajectory between two points in s and d per goal
    trajectories = []
    for target in target_set:
        s_target, d_target, t = target
        s_coef = quartic_polynomial_solver(s_start, s_target, t)
        d_coef = quintic_polynomial_solver(d_start, d_target, t)
        trajectories.append(tuple([s_coef, d_coef, t]))

    #evaluate and select "best" trajectory    
    best = min(trajectories, key=lambda tr: follow_cost(tr, vehicles))
   
    #return trajectories and best
    return trajectories, best



#Free Lane Change

def plan_lanechange(start_state, vehicles, obstacles = None):
    """
    Changing lanes with no vehicles around (no vehicle affecting the lane change)
    """
    print ('Maneuver: Lane Change')
    T = LC_TIME
    min_t = LC_MIN_TIME
    max_t = LC_MAX_TIME
    t_step = PLAN_TIME_STEP
    min_vel = MIN_VELOCITY
    max_vel = MAX_VELOCITY
    vel_step = VELOCITY_STEP 
    min_d = ROAD_R_BOUND 
    max_d_ = ROAD_L_BOUND
    d_step = ROAD_W_STEP

    s_start = start_state[:3]
    d_start = start_state[3:]
    target_set = []
    
    #generates alternative targets
    for t in np.arange(min_t, max_t, t_step):
        #longitudinal movement: goal is to keep velocity
            for vel in np.arange(min_vel, max_vel, vel_step):
                s_target = [0,0,0]
                s_target[2] = s_start[2]    # keep acceleratiion (?)
                s_target[1] = vel           # target velocity
                s_target[0] = s_start[0] + (vel * t) + s_start[2] * t**2 / 2.0   #predicted position
                #lateral movement
                for di in np.arange(min_d, max_d_, d_step):
                    d_target = [di,0,0] 
                    target_set.append((s_target,d_target,t))
            
    #fit jerk optimal trajectory between two points in s en d per goal
    trajectories = []
    for target in target_set:
        s_target, d_target, t = target
        s_coef = quartic_polynomial_solver(s_start, s_target, t)
        d_coef = quintic_polynomial_solver(d_start, d_target, t)
        trajectories.append(tuple([s_coef, d_coef, t]))

    #evaluate and select "best" trajectory    
    best = min(trajectories, key=lambda tr: follow_cost(tr, T, vehicles))
   
    #return trajectories and best
    return trajectories, best

#Cut-in Lane Change

def plan_cutin(start_state, delta, T, vehicles, target_id, var_time = False, var_pos = False, obstacles = None):
    s_start = start_state[:3]
    d_start = start_state[3:]
    
    multi_goals = []

    #main goal is relative to target vehicle predicted final position
    goal_state_relative = np.array( vehicles[target_id].state_in(T))  +  np.array(delta)
    s_goal = goal_state_relative[:3]
    d_goal = goal_state_relative[3:]
    print ('Cut-in optmized Goal around (' + str(T) +'):')
    print (goal_state_relative)

    #generate alternative goals in Time and Space
    if (var_time):
        time_step = PLAN_TIME_STEP
        t = T - 4 * time_step
        while t <= T + 4 * time_step:
            goal_state_relative = np.array(vehicles[target_id].state_in(t)) + np.array(delta)
            s_goal = goal_state_relative[:3]
            d_goal = goal_state_relative[3:]
            goal = [(s_goal, d_goal, t)]
            if(var_pos):  #generate alternative goals in Position
                for _ in range(N_SAMPLES):
                    goal = perturb_goal(s_goal, d_goal)
                    multi_goals.append((goal[0],goal[1],t))
            else:
                multi_goals += goal
            t +=time_step
    else:
        multi_goals += goal_state_relative
    

    #Fit Jerk minimal trajectory between two points in s en d per goal
    trajectories = []
    for goal in multi_goals:
        s_goal, d_goal, t = goal
        s_coef = QuinticPolynomialTrajectory(s_start, s_goal, t)
        d_coef = QuinticPolynomialTrajectory(d_start, d_goal, t)
        trajectories.append(tuple([s_coef, d_coef, t]))

    #evaluate and select "best" trajectory    
    
    #best = min(trajectories, key=lambda tr: calculate_cost(tr, target_id, delta, T, vehicles, FREE_LANNECHANGE_COST))
    best = min(trajectories, key=lambda tr: cutin_cost(tr, T, vehicles))
    #calculate again just to show in the terminal
    #calculate_cost(best, target_id, delta, T, vehicles, FREE_LANNECHANGE_COST, verbose=True)
   
    #return best
    return trajectories, best


#Single Trajectory (No optimization)
def plan_single_lanechange(start_state, goal_state, T):
    
    #generate a single goal
    s_start = start_state[:3]
    d_start = start_state[3:]
    s_goal = goal_state[:3]
    d_goal = goal_state[3:]
    print ('Lane Change Single Goal (' + str(T) +'):')
    print (goal_state)
    
    #Fit Jerk minimal trajectory between two points in s en d
    s_coeffs = QuinticPolynomialTrajectory(s_start, s_goal, T)
    d_coeffs = QuinticPolynomialTrajectory(d_start, d_goal, T)
    trajectory = tuple([s_coeffs, d_coeffs, T])
    #plot_single_trajectory(trajectory, None, False,True)
    return trajectory


def plan_single_cutin(start_state, T, delta, vehicles,  target_v_id,):

    s_start = start_state[:3]
    d_start = start_state[3:]
    
    goal_state_relative = np.array( vehicles[target_v_id].state_in(T))  +  np.array(delta)

    s_goal = goal_state_relative[:3]
    d_goal = goal_state_relative[3:]

    print ('Cut-In Single Goal (' + str(T) +'):')
    print (goal_state_relative)

    #Fit Jerk minimal trajectory between two points in s and d
    s_coeffs = QuinticPolynomialTrajectory(s_start, s_goal, T)
    d_coeffs = QuinticPolynomialTrajectory(d_start, d_goal, T)
    trajectory = tuple([s_coeffs, d_coeffs, T])
    return trajectory


#===POLYNOMIAL FITTING===


def quintic_polynomial_solver(start, end, T):
    """
    fits a jerk optimal quintic polynomial conecting start to end
    returns polynomial coeficients (alphas)
    """
    a_0, a_1, a_2 = start[0], start[1], start[2] / 2.0
    c_0 = a_0 + a_1 * T + a_2 * T**2
    c_1 = a_1 + 2* a_2 * T
    c_2 = 2 * a_2
    
    A = np.array([
                [T**3,   T**4,    T**5],
                [3*T**2, 4*T**3,  5*T**4],
                [6*T,   12*T**2, 20*T**3],
                ])
    B = np.array([
                end[0] - c_0,
                end[1] - c_1,
                end[2] - c_2
                ])
    a_3_4_5 = np.linalg.solve(A,B)
    alphas = np.concatenate([np.array([a_0, a_1, a_2]), a_3_4_5])
    return alphas

def quartic_polynomial_solver(start, end, T):
    """
    fits a jerk optimal quartic polynomial conecting start to end
    returns polynomial coeficients (alphas)
    """
    a_0, a_1, a_2 = start[0], start[1], start[2] / 2.0
    c_1 = a_1 + 2* a_2 * T
    c_2 = 2 * a_2
    
    A = np.array([
                [3*T**2, 4*T**3],
                [6*T,   12*T**2]
                ])
    B = np.array([
                end[1] - c_1,
                end[2] - c_2
                ])
    a_3_4 = np.linalg.solve(A,B)
    alphas = np.concatenate([np.array([a_0, a_1, a_2]), a_3_4])
    return alphas

#=== SAMPLE

#retunrs a perturbed version of the goal. 
#TODO: adapt
def perturb_goal(goal_s, goal_d):
    new_s_goal = []
    for mu, sig in zip(goal_s, SIGMA_S):
        new_s_goal.append(random.gauss(mu, sig))

    new_d_goal = []
    for mu, sig in zip(goal_d, SIGMA_D):
        new_d_goal.append(random.gauss(mu, sig))
        
    return tuple([new_s_goal, new_d_goal])



#deprecated #todelete
""" def OT_LaneChange(s_start, d_start, T, predictions, delta, target_vehicle):
    
    target = predictions[target_vehicle]
    
    # generate alternative goals in Time and Position
    #all_goals = []
    #timestep = 0.5
    #t = T - 4 * timestep
    #while t <= T + 4 * timestep:
    #    target_state = np.array(target.state_in(t)) + np.array(delta)
    #    goal_s = target_state[:3]
    #    goal_d = target_state[3:]
    #    goals = [(goal_s, goal_d, t)]
    #    for _ in range(N_SAMPLES):
    #        perturbed = perturb_goal(goal_s, goal_d)
    #        goals.append((perturbed[0], perturbed[1], t))
    #    all_goals += goals
    #    t += timestep
    #    print ('Goal (' + str(t) +')' )

    # generate alternative goals in Time Only
    all_goals = []
    timestep = 0.5
    t = T - 4 * timestep
    while t <= T + 4 * timestep:
        target_state = np.array(target.state_in(t)) + np.array(delta)
        goal_s = target_state[:3]
        goal_d = target_state[3:]
        goals = [(goal_s, goal_d, t)]
        all_goals += goals
        t += timestep
        print ('Goal (' + str(t) +')' )

    #generate a single goal
    #all_goals = []
    #target_state = np.array(target.state_in(T)) + np.array(delta)
    #goal_s = target_state[:3]
    #goal_d = target_state[3:]
    #all_goals = [(goal_s, goal_d, T)]
    #print ('Goal (' + str(T) +'):')
    #print  all_goals
    
    # find best trajectory
    trajectories = []
    for goal in all_goals:
        s_goal, d_goal, t = goal
        s_coefficients = QuinticPolynomialTrajectory(s_start, s_goal, t)
        d_coefficients = QuinticPolynomialTrajectory(d_start, d_goal, t)
        trajectories.append(tuple([s_coefficients, d_coefficients, t]))
    
    best = min(trajectories, key=lambda tr: calculate_cost(tr, target_vehicle, delta, T, predictions, WEIGHTED_COST_FUNCTIONS))
    #calculate again just to show in the terminal
    calculate_cost(best, target_vehicle, delta, T, predictions, WEIGHTED_COST_FUNCTIONS, verbose=True)
    show_trajectory(best[0], best[1], best[2], target_vehicle)
    for t in trajectories:
        show_trajectory(t[0], t[1], t[2])
    return best
 """
