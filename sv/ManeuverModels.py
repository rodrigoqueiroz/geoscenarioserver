#!/usr/bin/env python
#rqueiroz@gsd.uwaterloo.ca
# --------------------------------------------
# GEOSCENARIO Micro Maneuver Models for Motion Planning
# --------------------------------------------
import numpy as np
import random
import itertools
import datetime
import time
from dataclasses import dataclass
#from multiprocessing import Pool as ThreadPool
from sv.CostFunctions import *
from sv.ManeuverConfig import *
from util.Constants import *
from util.Utils import *

def plan_maneuver(man_key, mconfig, vehicle_frenet_state, lane_config, traffic_vehicles):
    #Micro maneuver layer
    if(man_key==M_VELKEEP):
        best, trajectories  = plan_velocity_keeping(vehicle_frenet_state, mconfig, lane_config, traffic_vehicles, None) 
    elif(man_key==M_STOP):
        best, trajectories  = plan_stop(vehicle_frenet_state, mconfig, lane_config, traffic_vehicles, None) 
    elif (man_key==M_FOLLOW):
        best, trajectories  = plan_following(vehicle_frenet_state, mconfig, lane_config, vehicles=traffic_vehicles)
    elif (man_key==M_LANESWERVE):
        best, trajectories = plan_laneswerve(vehicle_frenet_state,  mconfig, lane_config, traffic_vehicles, None) #returns tuple (s[], d[], t)
    #elif (man_key==CUTIN):
        #candidates, best = ST_CutIn( vehicle_frenet_state, delta, T,vehicles,target_id) #, True, True) #returns tuple (s[], d[], t)
        #candidates, best  = OT_CutIn( vehicle_frenet_state, delta, T, vehicles,target_id,True,True) #returns tuple (s[], d[], t)
    return best, trajectories

def plan_velocity_keeping(start_state, mconfig:MVelKeepConfig, lane_config:LaneConfig, vehicles = None, obstacles = None):
    """
    VELOCITY KEEPING
    Driving with no vehicle directly ahead
    No target point, but needs to adapt to a desired velocity
    """
    #print ('Maneuver: Velocity Keeping')
    s_start = start_state[:3]
    d_start = start_state[3:]
    target_t = mconfig.time.value
    target_vel = mconfig.vel.value
    min_d = lane_config.right_bound + VEHICLE_RADIUS
    max_d = lane_config.left_bound - VEHICLE_RADIUS
    d_samples = NUM_SAMPLING_D

    #generate alternative targets:
    target_state_set = []
    for t in mconfig.time.get_uniform_samples():
        #longitudinal movement: goal is to reach velocity
        for vel in mconfig.vel.get_uniform_samples():
            s_target = [0,vel,0] #pos not relevant
            #lateral movement
            for di in np.linspace(min_d, max_d, d_samples):
                d_target = [di,0,0] 
                #add target
                target_state_set.append((s_target,d_target,t))

    #print ('Targets: {}'.format(len(target_state_set)))

    #find trajectories
    trajectories = []
    trajectories = list(map(find_trajectory, zip(itertools.repeat(start_state), target_state_set)))  
    
    #evaluate feasibility
    #TODO: pre evaluate trajectories and eliminate unfeasible trajectories

    #cost
    best = min(trajectories, key=lambda x: velocity_keeping_cost(x, mconfig, lane_config, vehicles, obstacles))

    #return best trajectory, and candidates for debug
    return  best, list(trajectories)


def plan_following(start_state, mconfig:MFollowConfig, lane_config:LaneConfig, vehicles = None, obstacles = None):
    """ 
    VEHICLE FOLLOWING
    Moving target point, requiring a certain temporal safety distance to the vehicle ahead (constant time gap law).
    Predict leading vehicle (assume constant acceleration)
    """
    #print ('Maneuver:  vehicle following')

    s_start = start_state[:3]
    d_start = start_state[3:]
    target_vid = mconfig.target_vid
    target_t = mconfig.time.value
    time_gap = mconfig.time_gap
    #distance = mconfig.distance
    min_d = lane_config.right_bound + VEHICLE_RADIUS
    max_d = lane_config.left_bound - VEHICLE_RADIUS
    d_samples = NUM_SAMPLING_D
    
    #Pre conditions. Can the vehicle be followed?
    vehicle = vehicles[target_vid]
    #Is target ahead?
    if (s_start[0] >= vehicle.vehicle_state.s):
        return
    #Is target on the same lane?
    #TODO

    #generate alternative targets:
    target_state_set = []
    for t in mconfig.time.get_uniform_samples():
        #longitudinal movement: goal is to keep safe distance from leading vehicle
        s_lv = vehicle.future_state(t)[:3]
        s_target = [0,0,0]
        s_target[0] = s_lv[0] - (time_gap * s_lv[1])     
        s_target[1] = s_lv[1]                            
        s_target[2] = s_lv[2]                            
        #lateral movement
        for di in np.linspace(min_d, max_d, d_samples):
            d_target = [di,0,0] 
            #add target
            target_state_set.append((s_target,d_target,t))
        
    #find trajectories
    trajectories = []
    #zip two arrays for the pool.map
    trajectories = list(map(find_trajectory, zip(itertools.repeat(start_state), target_state_set)))  

    #evaluate feasibility
    #TODO: pre evaluate trajectories and eliminate unfeasible trajectories

     #cost
    best = min(trajectories, key=lambda x: follow_cost(x, mconfig, lane_config, vehicles, obstacles))
   
    #return best trajectory, and candidates for debug
    return  best, list(trajectories)


def plan_laneswerve(start_state, mconfig:MLaneSwerveConfig, lane_config, vehicles = None, obstacles = None):
    """
    LANE CHANGE SWERVE
    Swerve maneuver to another lane
    No vehicles affecting the lane change, except in case of collision detection (if Collision is on).
    """
    #print ('Maneuver: Lane Change Swerve')

    s_start = start_state[:3]
    d_start = start_state[3:]
    target_lid = mconfig.target_lid
    target_t = mconfig.time.value

    #Find target lane
    if (lane_config.id == target_lid):
        print('already in target lane {}'.format(target_lid))
        return None, None
    target_lane_config = lane_config.get_neighbour(target_lid)
    if not target_lane_config:
        print('target lane {} not found, is it a neighbour lane?'.format(target_lid))
        return None, None
    min_d = target_lane_config.right_bound + VEHICLE_RADIUS
    max_d = target_lane_config.left_bound - VEHICLE_RADIUS
    d_samples = NUM_SAMPLING_D
    
    #generate alternative targets:
    target_state_set = []
    for t in mconfig.time.get_uniform_samples():
        #longitudinal movement: goal is to keep velocity
        vel = s_start[1]
        s_target = [0,vel,0] #pos not relevant, no acc expected at the end
        #lateral movement
        for di in np.linspace(min_d, max_d, d_samples):
            d_target = [di,0,0] #no lateral movement expected at the end
            target_state_set.append((s_target,d_target,t))

    #find trajectories
    trajectories = []
    trajectories = list(map(find_trajectory, zip(itertools.repeat(start_state), target_state_set)))  #zip two arrays for the pool.map
    
    #cost
    best = min(trajectories, key=lambda x: laneswerve_cost(x, mconfig,lane_config,vehicles,obstacles))
    
    #return best trajectory, and candidates for debug
    return  best, list(trajectories)


def plan_cutin(start_state, delta, T, vehicles, target_id, var_time = False, var_pos = False, obstacles = None):
    """
    Cut-in Lane Change
    """ 
    s_start = start_state[:3]
    d_start = start_state[3:]
    
    multi_goals = []

    #main goal is relative to target vehicle predicted final position
    goal_state_relative = np.array( vehicles[target_id].future_state(T))  +  np.array(delta)
    s_goal = goal_state_relative[:3]
    d_goal = goal_state_relative[3:]
    print ('Cut-in optmized Goal around (' + str(T) +'):')
    print (goal_state_relative)

    #generate alternative goals in Time and Space
    if (var_time):
        time_step = PLAN_TIME_STEP
        t = T - 4 * time_step
        while t <= T + 4 * time_step:
            goal_state_relative = np.array(vehicles[target_id].future_state(t)) + np.array(delta)
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


def plan_stop(start_state, mconfig, lane_config, vehicles = None, obstacles = None):
    """
    STOP
    Stop can be a stop request by time, deceleration, or distance from current pos.
    """
    #print ('PLAN STOP')
    s_start = start_state[:3]
    d_start = start_state[3:]
    target_t = mconfig.time.value
    target_decel = mconfig.decel.value
    target_distance = mconfig.distance.value
    min_d = lane_config.right_bound + VEHICLE_RADIUS
    max_d = lane_config.left_bound - VEHICLE_RADIUS
    d_samples = 3
    #generate alternative targets:
    target_state_set = []
    for t in mconfig.time.get_uniform_samples():
        for dist in mconfig.distance.get_uniform_samples():
            #longitudinal movement: goal is to reach velocity aand acc 0
            s_pos = s_start[0] + dist
            s_target = [s_pos,0,0]
            #lateral movement
            for di in np.linspace(min_d, max_d, d_samples):
                d_target = [di,0,0] 
                #add target
                target_state_set.append((s_target,d_target,t))

    print ('Targets: {}'.format(len(target_state_set)))
    for i in target_state_set:
       print(i)
    
    #find trajectories
    trajectories = []
    #zip two arrays for the pool.map
    trajectories = list(map(find_trajectory, zip(itertools.repeat(start_state), target_state_set)))  
    
    #evaluate feasibility
    #TODO: pre evaluate trajectories and eliminate unfeasible trajectories

    #cost
    best = min(trajectories, key=lambda x: stop_cost(x, mconfig, lane_config, vehicles, obstacles))

    #return best trajectory, and candidates for debug
    return  best, list(trajectories)

def plan_stop_at(start_state, mconfig, lane_config, target_pos , vehicles = None, obstacles = None ):
    """
    STOP
    Stop can be a stop request by time and/or distance from current pos.
    Or optionally have a specific target position to stop (stop line, before an object, etc).
    """
    #print ('PLAN STOP')
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



#===POLYNOMIAL FITTING===


def find_trajectory(traj_bounds):
    """ 
    Fits a jerk optimal trajectory between two points in s and d.
    Returns a tuple representing the trajectory in the form of polynomial coefficients: 
    s_coef for quartic polynomial (longtudinal), d_coef for quintic polynomial (lateral), and time in [s].
    """
    unzip_traj_bounds = list(traj_bounds) 
    start_state = unzip_traj_bounds[0]
    target_state = unzip_traj_bounds[1]
    #print(start_state)
    #print(target)

    s_target, d_target, t = target_state
    s_start = start_state[:3]
    d_start = start_state[3:]
    s_coef = quartic_polynomial_solver(s_start, s_target, t)
    d_coef = quintic_polynomial_solver(d_start, d_target, t)
    return tuple([s_coef, d_coef, t])

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


    """
    #Threading Tests
    #check best target
    #_, d_coef, T = best
    #d_eq = to_equation(d_coef)
    #target_d = d_eq(T)
    #print('Best target is {:2f}'.format(target_d))

    #Multi Thread approach:
    pool = ThreadPool(4)
    trajectories = pool.starmap(find_trajectory, zip(itertools.repeat(start_state), target_state_set)
    pool.close()
    pool.join()
    
    pool = ThreadPool(4)
    #trajectories_cost = map(velocity_keeping_cost,zip(trajectories,itertools.repeat(vehicles))) #ST
    trajectories_cost = pool.map(velocity_keeping_cost, zip(trajectories,itertools.repeat(vehicles)))
    trajectories_cost = list(trajectories_cost)
    pool.close()
    pool.join()
    trajectories_cost = list(trajectories_cost)
    best_cost = min(trajectories_cost, key=lambda t: t[1])
    best = best_cost[0]
    """

    #DEBUG ONLY:

def plan_single_lanechange(start_state, goal_state, T):
    """
    Single Trajectory (No optimization)
    """
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
    return trajectory


def plan_single_cutin(start_state, T, delta, vehicles,  target_v_id,):

    s_start = start_state[:3]
    d_start = start_state[3:]
    
    goal_state_relative = np.array( vehicles[target_v_id].future_state(T))  +  np.array(delta)

    s_goal = goal_state_relative[:3]
    d_goal = goal_state_relative[3:]

    print ('Cut-In Single Goal (' + str(T) +'):')
    print (goal_state_relative)

    #Fit Jerk minimal trajectory between two points in s and d
    s_coeffs = QuinticPolynomialTrajectory(s_start, s_goal, T)
    d_coeffs = QuinticPolynomialTrajectory(d_start, d_goal, T)
    trajectory = tuple([s_coeffs, d_coeffs, T])
    return trajectory
