#!/usr/bin/env python
#rqueiroz@gsd.uwaterloo.ca
# ---------------------------------------------
# TRAJECTORY EVALUATION
# --------------------------------------------
#TODO: configurable weights (as constants?)
#TODO: add alternative total cost functions for agressive maneuvers leading to crash
#TODO: add distance to central lane cost
#TODO: add driver error to cost

import numpy as np
from Utils import *
from Constants import *

# FEASIBILITY

def check_trajectory(trajectory, vehicles, allow_near_collision):
    #TODO: adjust to support near collision under certain conditions 
    if (collision_cost(trajectory, vehicles)):
        return False
    return True


#MANEUVER COST
#Stopping should avoid change in long acceleration
def stop_cost(trajectory, vehicles):
    total_cost = 0
    C = []
    C.append(99 * collision_cost(trajectory, vehicles))
    C.append(1 * total_jerk_cost(trajectory))
    C.append(1 * max_acc_cost(trajectory))
    C.append(1 * total_acc_cost(trajectory))
    #c.append(1 * time_cost(trajectory, goal_t))
    #c.append(1 * total_lat_jerk_cost(trajectory))
    #c.append(1 * max_lat_jerk_cost(trajectory))
    
    
    #TODO: Add cost to center line distance
    #TODO: Add cost to change trajectory (to make keeping the same trajectory easier)

    #KD * tfp.d[-1]**2
    #ds = (TARGET_SPEED - tfp.s_d[-1])**2
    #KD * ds

    total_cost = sum(C)
    #print("TOTAL COST: " + str(total_cost) + " " + str(C))
    return total_cost

def velocity_keeping_cost(trajectory, vehicles):
    total_cost = 0
    C = []
    C.append(99 * collision_cost(trajectory, vehicles))
    #C.append(1 * time_cost(trajectory, goal_t))
    C.append(1 * total_lat_jerk_cost(trajectory))
    C.append(1 * max_lat_jerk_cost(trajectory))
    C.append(1 * max_acc_cost(trajectory))
    C.append(1 * total_acc_cost(trajectory))
    
    #TODO: Add cost to center line distance
    #TODO: Add cost to change trajectory (to make keeping the same trajectory easier)

    #KD * tfp.d[-1]**2
    #ds = (TARGET_SPEED - tfp.s_d[-1])**2
    #KD * ds

    total_cost = sum(C)
    #print("TOTAL COST: " + str(total_cost) + " " + str(C))
    return total_cost

def follow_cost(trajectory, vehicles):
    total_cost = 0
    C = []
    C.append(99 * collision_cost(trajectory, vehicles))
    #C.append(1 * time_cost(trajectory, goal_t))
    C.append(1 * total_lat_jerk_cost(trajectory))
    C.append(1 * max_lat_jerk_cost(trajectory))
    C.append(1 * max_acc_cost(trajectory))
    C.append(1 * total_acc_cost(trajectory))
    
    #TODO: Add cost to center line distance
    #TODO: Add cost to change trajectory (to make keeping the same trajectory easier)

    #KD * tfp.d[-1]**2
    #ds = (TARGET_SPEED - tfp.s_d[-1])**2
    #KD * ds

    total_cost = sum(C)
    #print("TOTAL FL COST: " + str(total_cost) + " " + str(C))
    return total_cost


def cutin_cost(trajectory, goal_t, vehicles):
    total_cost = 0
    C = []
    C.append(3 * collision_cost(trajectory, vehicles))
    C.append(1 * time_cost(trajectory, goal_t))
    C.append(1 * total_jerk_cost(trajectory))
    C.append(1 * max_jerk_cost(trajectory))
    C.append(1 * max_acc_cost(trajectory))
    C.append(1 * total_acc_cost(trajectory))
    total_cost = sum(C)
    #print("TOTAL CI COST: " + str(total_cost) + " " + str(C))
    return total_cost

def lanechange_cost(trajectory, goal_t, vehicles):
    total_cost = 0
    C = []
    C.append(3 * collision_cost(trajectory, vehicles))
    C.append(1 * time_cost(trajectory, goal_t))
    C.append(1 * total_jerk_cost(trajectory))
    C.append(1 * max_jerk_cost(trajectory))
    C.append(1 * max_acc_cost(trajectory))
    C.append(1 * total_acc_cost(trajectory))
    total_cost = sum(C)
    #print("TOTAL LC COST: " + str(total_cost) + " " + str(C))
    return total_cost

# INDIVIDUAL COST FUNCTIONS

#Penalizes trajectories longer or shorter than target time
def time_cost(trajectory, T):
    s, d, t = trajectory
    return logistic(float(abs(t-T)) / T)

#Penalizes collision
def collision_cost(trajectory, vehicles):
    if (vehicles == None):
        return 0.0
    nearest = nearest_approach_to_any_vehicle(trajectory, vehicles)
    if nearest < 2*VEHICLE_RADIUS: return 1.0
    else : return 0.0

#JERK
def total_jerk_cost(traj):
    s, d, T = traj
    s_dot = differentiate(s)
    s_d_dot = differentiate(s_dot)
    jerk = to_equation(differentiate(s_d_dot))
    total_jerk = 0
    dt = float(T) / 100.0
    for i in range(100):
        t = dt * i
        j = jerk(t)
        total_jerk += abs(j*dt)
    jerk_per_second = total_jerk / T
    return logistic(jerk_per_second / EXPECTED_JERK_IN_ONE_SEC )

def max_jerk_cost(traj):
    s, d, T = traj
    s_dot = differentiate(s)
    s_d_dot = differentiate(s_dot)
    jerk = differentiate(s_d_dot)
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
    return logistic(jerk_per_second / EXPECTED_JERK_IN_ONE_SEC )

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
    
    return logistic(acc_per_second / EXPECTED_ACC_IN_ONE_SEC )
    
def max_acc_cost(trajectory):
    s, d, T = trajectory
    s_dot = differentiate(s)
    s_d_dot = differentiate(s_dot)
    a = to_equation(s_d_dot)
    all_accs = [a(float(T)/100 * i) for i in range(100)]
    max_acc = max(all_accs, key=abs)
    if abs(max_acc) > MAX_ACCEL: return 1
    else: return 0

#penalizes distance from central line
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

#Penalizes lower average speeds.
def effic_cost(trajectory,vehicles):
    s, _, t = trajectory
    s = to_equation(s)
    avg_v = float(s(t)) / t
    return logistic(2*float(targ_v - avg_v) / avg_v)



#CHECK:
def time_diff_cost(traj, target_vehicle, delta, T, predictions):
    """
    Penalizes trajectories that span a duration which is longer or 
    shorter than the duration requested.
    """
    _, _, t = traj
    return logistic(float(abs(t-T)) / T)

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
    
def stays_on_road_cost(traj, target_vehicle, delta, T, predictions):
    pass

def exceeds_speed_limit_cost(traj, target_vehicle, delta, T, predictions):
    pass

def efficiency_cost(traj, target_vehicle, delta, T, predictions):
    """
    Rewards high average speeds.
    """
    s, _, t = traj
    s = to_equation(s)
    avg_v = float(s(t)) / t
    targ_s, _, _, _, _, _ = predictions[target_vehicle].state_in(t)
    targ_v = float(targ_s) / t
    return logistic(2*float(targ_v - avg_v) / avg_v)

def total_acc_cost(traj):
    s, d, T = traj
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
    
    return logistic(acc_per_second / EXPECTED_ACC_IN_ONE_SEC )
    
def max_acc_cost(traj):
    s, d, T = traj
    s_dot = differentiate(s)
    s_d_dot = differentiate(s_dot)
    a = to_equation(s_d_dot)
    all_accs = [a(float(T)/100 * i) for i in range(100)]
    max_acc = max(all_accs, key=abs)
    if abs(max_acc) > MAX_ACCEL: return 1
    else: return 0
    


