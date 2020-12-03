#!/usr/bin/env python
#rqueiroz@uwaterloo.ca
# ---------------------------------------------
# TRAJECTORY EVALUATION
# --------------------------------------------

import numpy as np
from sv.ManeuverConfig import *
from util.Utils import *
from SimConfig import *

#=============================== FEASIBILITY

def maneuver_feasibility(start_state, trajectory,  mconfig, lane_config:LaneConfig, vehicles, obstacles):

    for c in mconfig.feasibility_constraints:
        if mconfig.feasibility_constraints[c] > 0:
            #collision
            if c == 'collision':
                if collision_cost(start_state, trajectory, lane_config, vehicles) > 0:
                    return False
            #off road
            elif c == 'off_lane':
                if off_lane_cost(trajectory, lane_config) > 0:
                    return False
            #acc
            elif c == 'max_long_acc':
                if max_acc_cost(trajectory, mconfig.max_acc) > 0:
                    return False
            elif c == 'max_lat_acc':
                if max_acc_cost(trajectory, mconfig.max_lat_acc) > 0:
                    return False
            #jerk
            elif c == 'max_lat_jerk':
                if max_lat_jerk_cost(trajectory, mconfig.max_lat_jerk) > 0:
                    return False
            elif c == 'max_long_jerk':
                if max_long_jerk_cost(trajectory, mconfig.max_long_jerk) > 0:
                    return False
            else:
                raise NotImplementedError("Feasibility function {} not implemented".format(c))
    return True

#=============================== MANEUVER COST

def maneuver_cost(start_state, trajectory, target_state, mconfig, lane_config:LaneConfig, vehicles, obstacles):
    C = []
    for cost in mconfig.cost_weight:
        k = mconfig.cost_weight[cost]
        if k > 0:
            #basic cost
            if (cost == 'time_cost'):
                if hasattr(mconfig, 'time'):
                    C.append( k * time_cost(trajectory, mconfig.time.value))
            elif cost == 'effic_cost':
                if hasattr(mconfig ,'vel'):
                    C.append( k * effic_cost(trajectory, mconfig.vel.value))
            #goal
            elif cost == 'progress_cost':
                C.append( k * progress_cost(start_state, trajectory, target_state))
            elif cost == 'lane_offset_cost':
                C.append( k * lane_offset_cost(trajectory, lane_config, mconfig.expected_offset_per_sec))
            #jerk
            elif cost == 'total_lat_jerk_cost':
                C.append( k * total_lat_jerk_cost(trajectory, mconfig.expected_lat_jerk_per_sec))
            elif cost == 'total_long_jerk_cost':
                C.append( k * total_long_jerk_cost(trajectory, mconfig.expected_long_jerk_per_sec))
            #acc
            elif cost == 'total_long_acc_cost':
                C.append( k * total_long_acc_cost(trajectory, mconfig.expected_long_acc_per_sec))
            elif cost == 'total_lat_acc_cost':
                C.append( k * total_lat_acc_cost(trajectory, mconfig.expected_lat_acc_per_sec))
            #proximity
            elif cost == 'proximity_cost':
                C.append( k * proximity_cost(start_state, trajectory, lane_config, vehicles ))
            else:
                raise NotImplementedError("Cost function {} not implemented".format(cost))
    return sum(C)

#=============================== INDIVIDUAL COST FUNCTIONS


#Efficiency Cost:

def effic_cost(trajectory,target_vel):
    '''Penalizes low average velocity'''
    s_coef, _, T = trajectory
    vel = to_equation(differentiate(s_coef))
    total = 0
    dt = float(T) / 100.0
    for ti in range(100):
        total += vel(ti)
    avg_vel = float(total) / 100
    return logistic( (target_vel - avg_vel) / avg_vel)

def time_cost(trajectory, target_t):
    '''Penalizes trajectories longer or shorter than target time.'''
    _, _, T = trajectory
    diff =  float(abs(T-target_t))
    return logistic( diff / target_t)

def progress_cost(start_state, trajectory, target_state):
    '''Penalizes trajectories that end away from the target state'''
    s_coef, d_coef, T = trajectory
    # TODO add velocity and acc comparisons also?
    end_arr = np.array([
        to_equation(s_coef)(T),
        to_equation(d_coef)(T)
    ])
    target_arr = np.array([
        target_state[0][0],
        target_state[1][0]
    ])
    diff = end_arr - target_arr
    if diff[0] < 0:
        # harshly penalize overshooting trajectories
        cost = 1
    else:
        rel_error = np.linalg.norm() / np.linalg.norm(target_arr)
        cost = logistic(rel_error)
    # print("target stop {}, actual {}, cost {}".format(
    #     target_state[0][0],
    #     end_arr[0],
    #     cost
    # ))
    return cost

#Road geometry Cost:

def lane_offset_cost(trajectory,lane_config, expected_offset_per_sec):
    '''Penalizes distance from lane center during the entire trajectory'''
    _, d_coef, T = trajectory

    central_d = (lane_config.left_bound - lane_config.right_bound)/2 + lane_config.right_bound

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
    cost = logistic(offset_per_second/expected_offset_per_sec)

    #print("total lateral offset cost is {:.2f} for target {:.2f}".format(cost,target_d) )
    return cost

def off_lane_cost(trajectory,lane_config):
    '''
    Penalizes trajectories going beyond lane boundaries
    Current implementation relies on a fixed lane width.
    #TODO: Adapt to project lane over the entire trajectory
    '''
    _, d_coef, T = trajectory
    d_eq = to_equation(d_coef)
    dt = float(T) / 100.0
    for i in range(100):
        t = dt * i
        d = d_eq(t)
        if not (lane_config.left_bound > d > lane_config.right_bound):
            #print("off_lane")
            return 1
    return 0


#Jerk Cost:

def max_long_jerk_cost(trajectory, expected_max_jerk):
    '''Penalizes high longitudinal jerk (binary function)'''
    s_coef, _, T = trajectory
    jerk = to_equation(differentiate(differentiate(differentiate(s_coef))))
    all_jerks = [jerk(float(T)/100 * i) for i in range(100)]
    max_jerk = max(all_jerks, key=abs)
    if abs(max_jerk) > expected_max_jerk:
        return 1
    else:
        return 0

def max_lat_jerk_cost(trajectory, expected_max_jerk):
    '''Penalizes high lateral jerk (binary function)'''
    _, d_coef, T = trajectory
    jerk = to_equation(differentiate(differentiate(differentiate(d_coef))))
    all_jerks = [jerk(float(T)/100 * i) for i in range(100)]
    max_jerk = max(all_jerks, key=abs)
    if abs(max_jerk) > expected_max_jerk:
        return 1
    else:
        return 0

def total_long_jerk_cost(trajectory, expected_jerk_per_sec):
    '''Penalizes high longitudinal jerk '''
    s_coef, _, T = trajectory
    jerk = to_equation(differentiate(differentiate(differentiate(s_coef))))
    total_jerk = 0
    dt = float(T) / 100.0
    for i in range(100):
        t = dt * i
        j = jerk(t)
        total_jerk += abs(j*dt)
    jerk_per_second = total_jerk / T
    return logistic(jerk_per_second / expected_jerk_per_sec )

def total_lat_jerk_cost(trajectory, expected_jerk_per_sec):
    '''Penalizes high lateral jerk '''
    _, d_coef, T = trajectory
    jerk = to_equation(differentiate(differentiate(differentiate(d_coef))))
    total_jerk = 0
    dt = float(T) / 100.0
    for i in range(100):
        t = dt * i
        j = jerk(t)
        total_jerk += abs(j*dt)
    jerk_per_second = total_jerk / T
    return logistic(jerk_per_second / expected_jerk_per_sec)

#Acc
def max_long_acc_cost(trajectory, expected_max_acc):
    '''Penalizes high longitudinal acceleration (binary function)'''
    s_coef, _, T = trajectory
    return max_acc_cost(s_coef, T, expected_max_acc)

def max_lat_acc_cost(trajectory, expected_max_acc):
    '''Penalizes high lateral acceleration (binary function)'''
    _, d_coef, T = trajectory
    return max_acc_cost(d_coef, T, expected_max_acc)

def max_acc_cost(x_coef, T, expected_acc_per_sec):
    acc = to_equation(differentiate(differentiate(x_coef)))
    all_accs = []
    dt = float(T) / 100.0
    for i in range(100):
        t = dt * i
        all_accs.append(acc(t))
    max_acc = max(all_accs, key=abs)
    if abs(max_acc) > expected_max_acc:
        return 1
    else:
        return 0

def total_long_acc_cost(trajectory, expected_long_acc_per_sec):
    '''Penalizes high longitudinal acceleration'''
    s_coef, _, T = trajectory
    return total_acc_cost(s_coef, T, expected_long_acc_per_sec)

def total_lat_acc_cost(trajectory, expected_lat_acc_per_sec):
    '''Penalizes high lateral acceleration'''
    _, d_coef, T = trajectory
    return total_acc_cost(d_coef, T, expected_lat_acc_per_sec)

def total_acc_cost(x_coef, T, expected_acc_per_sec):
    acc = to_equation(differentiate(differentiate(x_coef)))
    total_acc = 0
    dt = float(T) / 100.0
    for i in range(100):
        t = dt * i
        a = acc(t)
        total_acc += abs(a*dt)
    acc_per_sec = total_acc / T
    return logistic(acc_per_sec / expected_acc_per_sec )


#Proximity Cost:

def collision_cost(start_state, trajectory, lane_config, vehicles):
    '''Penalizes collision (binary function)'''
    if (vehicles == None):
        return 0
    #check nearest
    nearest = nearest_to_vehicles_ahead(start_state, trajectory, lane_config, vehicles)
    if nearest < 2*VEHICLE_RADIUS:
        return 1
    else:
        return 0

def proximity_cost(start_state, trajectory, lane_config, vehicles):
    '''Penalizes proximity to other vehicles'''
    if (vehicles == None):
        return 0.0
    #check nearest
    nearest = nearest_to_vehicles_ahead(start_state, trajectory, lane_config, vehicles)
    return logistic(2*VEHICLE_RADIUS / nearest)


def nearest_to_vehicles_ahead(start_state, trajectory, lane_config, vehicles):
    '''Calculates the closest distance to any vehicle ahead during a trajectory'''
    closest = 999999
    s = start_state[0]
    d = start_state[3]
    left = lane_config.left_bound
    right = lane_config.right_bound
    for k, v in vehicles.items():
        #if close
        if ( abs(v.vehicle_state.s - s) < 40):
            #if same lane
            if (left > v.vehicle_state.d > right):
                #if ahead
                if (v.vehicle_state.s > s):
                    #print("vehicle {} is ahead".format(k))
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
        targ_s, _, _, targ_d, _, _ = vehicle.future_state(t)
        dist = sqrt((cur_s-targ_s)**2 + (cur_d-targ_d)**2)
        if dist < closest:
            closest = dist
    return closest
