#!/usr/bin/env python
#rqueiroz@gsd.uwaterloo.ca
# ---------------------------------------------
# TRAJECTORY EVALUATION
# --------------------------------------------
#TODO: configurable weights (as constants?)
#TODO: add alternative total cost functions for agressive maneuvers leading to crash
#TODO: add driver error to cost
#TODO: Add cost to change trajectory (to make keeping the same trajectory easier)
#TODO: Add traj feasibility check

import numpy as np
from Utils import *
from Constants import *
from ManeuverConfig import *

#=============================== FEASIBILITY

def check_trajectory(trajectory, vehicles, allow_near_collision):
    #TODO: adjust to support near collision under certain conditions 
    if (collision_cost(trajectory, vehicles)):
        return False
    return True

#=============================== MANEUVER COST
def velocity_keeping_cost(trajectory, mconfig:MVelKeepConfig, lane_config, vehicles, obstacles):
    total_cost = 0
    C = []
    C.append(1 * time_cost(trajectory, mconfig.time.value))
    C.append(1 * lateral_lane_offset_cost(trajectory,lane_config))
    #C.append(1 * total_lat_jerk_cost(trajectory))
    #C.append(1 * max_lat_jerk_cost(trajectory))
    #C.append(1 * max_acc_cost(trajectory))
    #C.append(1 * total_acc_cost(trajectory))
    #C.append(99 * collision_cost(trajectory, vehicles))
    total_cost = sum(C)
    return total_cost

def follow_cost(trajectory, mconfig:MFollowConfig, lane_config, vehicles, obstacles):
    total_cost = 0
    C = []
    C.append(1 * time_cost(trajectory, mconfig.time.value))
    C.append(1 * lateral_lane_offset_cost(trajectory,lane_config))
    #C.append(1 * total_lat_jerk_cost(trajectory))
    #C.append(1 * max_lat_jerk_cost(trajectory))
    #C.append(1 * max_acc_cost(trajectory))
    #C.append(1 * total_acc_cost(trajectory))
    #C.append(99 * collision_cost(trajectory, vehicles))
    total_cost = sum(C)
    return total_cost

def laneswerve_cost(trajectory, mconfig:MLaneSwerveConfig, lane_config, vehicles, obstacles):
    total_cost = 0
    C = []
    C.append(1 * time_cost(trajectory, mconfig.time.value))
    C.append(1 * lateral_lane_offset_cost(trajectory,lane_config))
    #C.append(1 * time_cost(trajectory, goal_t))
    #C.append(1 * total_lat_jerk_cost(trajectory))
    #C.append(1 * max_lat_jerk_cost(trajectory))
    #C.append(1 * max_acc_cost(trajectory))
    #C.append(1 * total_acc_cost(trajectory))
    #C.append(3 * collision_cost(trajectory, vehicles))
    total_cost = sum(C)
    return total_cost

def cutin_cost(trajectory, mconfig:MCutInConfig, lane_config, vehicles, obstacles):
    total_cost = 0
    C = []
    C.append(1 * time_cost(trajectory, mconfig.time.value))
    C.append(1 * lateral_lane_offset_cost(trajectory,lane_config))
    #C.append(1 * time_cost(trajectory, goal_t))
    #C.append(1 * total_jerk_cost(trajectory))
    #C.append(1 * max_jerk_cost(trajectory))
    #C.append(1 * max_acc_cost(trajectory))
    #C.append(1 * total_acc_cost(trajectory))
    #C.append(3 * collision_cost(trajectory, vehicles))
    total_cost = sum(C)
    return total_cost

#Stopping should avoid change in long acceleration
def stop_cost(trajectory, mconfig, lane_config, vehicles, obstacles):
    total_cost = 0
    C = []
    C.append(1 * time_cost(trajectory, mconfig.time.value))
    C.append(1 * lateral_lane_offset_cost(trajectory,lane_config))
    #C.append(1 * total_jerk_cost(trajectory))
    #C.append(1 * max_acc_cost(trajectory))
    #C.append(1 * total_acc_cost(trajectory))
    #C.append(1 * time_cost(trajectory, goal_t))
    #C.append(1 * total_lat_jerk_cost(trajectory))
    #C.append(1 * max_lat_jerk_cost(trajectory))
    #C.append(99 * collision_cost(trajectory, vehicles))
    total_cost = sum(C)
    #print("TOTAL COST: " + str(total_cost) + " " + str(C))
    return total_cost
#=============================== INDIVIDUAL COST FUNCTIONS


def time_cost(trajectory, target_t):
    #Penalizes trajectories longer or shorter than target time.
    _, _, t = trajectory
    diff =  float(abs(t-target_t))
    return logistic( diff / target_t)

def lateral_lane_offset_cost(traj,lane_config):
    #Penalizes distance from lane center during the entire trajectory.
    #Not suitable for Lane Change
    _, d_coef, T = traj
    central_d = (lane_config.left_bound - lane_config.right_bound)/2
    
    d_eq = to_equation(d_coef)
    target_d = d_eq(T)
    total_offset = 0

    dt = float(T) / 100.0
    for i in range(100):
        t = dt * i
        offset = d_eq(t) - central_d
        total_offset += abs(offset)
        #print("dt {:.2f} central_d {:.2f} offset {:.2f} ".format(d_eq(t),central_d,offset))
    
    #print("total offset is {:.2f} ".format(total_offset))
    offset_per_second = total_offset / T
    #print("offset_per_second {:.2f}".format(offset_per_second) )
    cost = logistic(offset_per_second/EXPECTED_OFFSET_PER_SEC)
    
    #print("total lateral offset cost is {:.2f} for target {:.2f}".format(cost,target_d) )
    return cost


def effic_cost(trajectory,target_vel):
    #Penalizes lower average velocity.
    s, _, T = trajectory
    vel_coef = differentiate(s)
    vel_eq = to_equation(vel_coef)
    dt = float(T) / 100.0
    for ti in range(100):
        total += vel_eq(ti)
    avg_vel = float(total) / 100
    return logistic( (target_vel - avg_vel) / avg_vel)

#Jerk

def total_long_jerk_cost(trajectory):
    s, _, T = trajectory
    s_d = differentiate(s)
    s_dd = differentiate(s_d)
    jerk = to_equation(differentiate(s_dd))
    total_jerk = 0
    dt = float(T) / 100.0
    for i in range(100):
        t = dt * i
        j = jerk(t)
        total_jerk += abs(j*dt)
    jerk_per_second = total_jerk / T
    return logistic(jerk_per_second / EXPECTED_JERK_PER_SEC )


def max_long_jerk_cost(trajectory):
    s, _, T = trajectory
    s_d = differentiate(s)
    s_d = differentiate(s_d)
    jerk = differentiate(s_dd)
    jerk = to_equation(jerk)
    all_jerks = [jerk(float(T)/100 * i) for i in range(100)]
    max_jerk = max(all_jerks, key=abs)
    if abs(max_jerk) > MAX_JERK: return 1
    else: return 0

def total_lat_jerk_cost(trajectory):
    _, d, T = trajectory
    d_d = differentiate(d)
    d_dd = differentiate(d_d)
    jerk = to_equation(differentiate(d_dd))
    total_jerk = 0
    dt = float(T) / 100.0
    for i in range(100):
        t = dt * i
        j = jerk(t)
        total_jerk += abs(j*dt)
    jerk_per_second = total_jerk / T
    return logistic(jerk_per_second / EXPECTED_JERK_PER_SEC)

def max_lat_jerk_cost(trajectory):
    _, d, T = trajectory
    d_d = differentiate(d)
    d_dd = differentiate(d_d)
    jerk = differentiate(d_dd)
    jerk = to_equation(jerk)
    all_jerks = [jerk(float(T)/100 * i) for i in range(100)]
    max_jerk = max(all_jerks, key=abs)
    if abs(max_jerk) > MAX_JERK: return 1
    else: return 0

#Acc
def total_acc_cost(trajectory):
    s, d, T = trajectory
    s_dot = differentiate(s)
    s_d_dot = differentiate(s_dot)
    a = to_equation(s_d_dot)
    total_acc = 0
    dt = float(T) / 100.0
    for i in range(100):
        t = dt * i
        acc = a(t)
        total_acc += abs(acc*dt)
    acc_per_second = total_acc / T
    
    return logistic(acc_per_second / EXPECTED_ACC_PER_SEC )
    
def max_acc_cost(trajectory):
    s, d, T = trajectory
    s_dot = differentiate(s)
    s_d_dot = differentiate(s_dot)
    a = to_equation(s_d_dot)
    all_accs = [a(float(T)/100 * i) for i in range(100)]
    max_acc = max(all_accs, key=abs)
    if abs(max_acc) > MAX_ACCEL: return 1
    else: return 0


#Penalizes collision
def collision_cost(trajectory, vehicles):
    if (vehicles == None):
        return 0.0
    nearest = nearest_approach_to_any_vehicle(trajectory, vehicles)
    if nearest < 2*VEHICLE_RADIUS: return 1.0
    else : return 0.0


#Calculates the closest distance to any vehicle during a trajectory.
def nearest_approach_to_any_vehicle(trajectory, vehicles):
    closest = 999999
    for v in vehicles.values():
        d = nearest_approach(trajectory,v)
        if d < closest:
            closest = d
    return closest

def nearest_approach(trajectory, vehicle):
    closest = 999999
    s_,d_,T = trajectory
    s = to_equation(s_)
    d = to_equation(d_)
    for i in range(100):
        t = float(i) / 100 * T
        cur_s = s(t)
        cur_d = d(t)
        targ_s, _, _, targ_d, _, _ = vehicle.state_in(t)
        dist = sqrt((cur_s-targ_s)**2 + (cur_d-targ_d)**2)
        if dist < closest:
            closest = dist
    return closest

#CHECK:
def s_diff_cost(traj, target_vehicle, delta, T, predictions):
    """
    Penalizes trajectories whose s coordinate (and derivatives) 
    differ from the goal.
    """
    s, _, T = traj
    target = predictions[target_vehicle].state_in(T)
    target = list(np.array(target) + np.array(delta))
    s_targ = target[:3]
    S = [f(T) for f in get_f_and_N_derivatives(s, 2)]
    cost = 0
    for actual, expected, sigma in zip(S, s_targ, SIGMA_S):
        diff = float(abs(actual-expected))
        cost += logistic(diff/sigma)
    return cost

def d_diff_cost(traj, target_vehicle, delta, T, predictions):
    """
    Penalizes trajectories whose d coordinate (and derivatives) 
    differ from the goal.
    """
    _, d_coeffs, T = traj
    
    d_dot_coeffs = differentiate(d_coeffs)
    d_ddot_coeffs = differentiate(d_dot_coeffs)

    d = to_equation(d_coeffs)
    d_dot = to_equation(d_dot_coeffs)
    d_ddot = to_equation(d_ddot_coeffs)

    D = [d(T), d_dot(T), d_ddot(T)]
    
    target = predictions[target_vehicle].state_in(T)
    target = list(np.array(target) + np.array(delta))
    d_targ = target[3:]
    cost = 0
    for actual, expected, sigma in zip(D, d_targ, SIGMA_D):
        diff = float(abs(actual-expected))
        cost += logistic(diff/sigma)
    return cost


def buffer_cost(traj, target_vehicle, delta, T, predictions):
    """
    Penalizes getting close to other vehicles.
    """
    nearest = nearest_approach_to_any_vehicle(traj, predictions)
    return logistic(2*VEHICLE_RADIUS / nearest)



