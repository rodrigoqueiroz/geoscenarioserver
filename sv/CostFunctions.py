#!/usr/bin/env python
#rqueiroz@uwaterloo.ca
# ---------------------------------------------
# TRAJECTORY EVALUATION
# --------------------------------------------

import numpy as np
from sv.ManeuverConfig import *
from util.Utils import *
from SimConfig import *
from sp.Pedestrian import *

#=============================== FEASIBILITY

def maneuver_feasibility(start_state, trajectory, target_state, mconfig, lane_config:LaneConfig, vehicles, pedestrians, static_objects):

    for c in mconfig.feasibility_constraints:
        if mconfig.feasibility_constraints[c] > 0:
            #collision
            if c == 'collision':
                if collision_cost(start_state, trajectory, lane_config, vehicles,pedestrians,static_objects) > 0:
                    return False
            #off road
            elif c == 'off_lane':
                if off_lane_cost(trajectory, lane_config) > 0:
                    return False
            #acc
            elif c == 'max_long_acc':
                if max_long_acc_cost(trajectory, mconfig.max_long_acc) > 0:
                    return False
            elif c == 'max_lat_acc':
                if max_lat_acc_cost(trajectory, mconfig.max_lat_acc) > 0:
                    return False
            #jerk
            elif c == 'max_lat_jerk':
                if max_lat_jerk_cost(trajectory, mconfig.max_lat_jerk) > 0:
                    return False
            elif c == 'max_long_jerk':
                if max_long_jerk_cost(trajectory, mconfig.max_long_jerk) > 0:
                    return False
            elif c == 'direction':
                if direction_cost(trajectory, start_state, target_state) > 0:
                    return False
            else:
                raise NotImplementedError("Feasibility function {} not implemented".format(c))
    return True

#=============================== MANEUVER COST

def maneuver_cost(start_state, trajectory, target_state, mconfig, lane_config:LaneConfig, vehicles, pedestrians, static_objects):
    C = []
    prox_cost = 0
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
                C.append(k * proximity_cost(start_state, trajectory, lane_config, vehicles, pedestrians, static_objects ))
            else:
                raise NotImplementedError("Cost function {} not implemented".format(cost))
    return sum(C)

#=============================== INDIVIDUAL COST FUNCTIONS


#Efficiency Cost:

def effic_cost(trajectory,target_vel):
    '''Penalizes low average velocity'''
    s_coef, _, T = trajectory
    fs = to_equation(s_coef)
    distance = (fs(T) - fs(0))
    avg_vel = distance / T
    #method2:
    #total = 0
    #dt = float(T) / 100.0
    #for ti in range(100):
    #    total += vel(ti)
    #avg_vel = float(total) / 100
    return logistic( 2*(target_vel-avg_vel) / avg_vel)

def time_cost(trajectory, target_t):
    '''Penalizes trajectories longer or shorter than target time.'''
    _, _, T = trajectory
    diff =  float(abs(T-target_t))
    return logistic( diff / target_t)

def progress_cost(start_state, trajectory, target_state):
    '''Penalizes trajectories that end away from the target state.
    NOTE: This may not applyto stop_at anymore!
    '''
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
        rel_error = np.linalg.norm(diff) / np.linalg.norm(target_arr)
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

def max_acc_cost(x_coef, T, expected_max_acc):
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

def direction_cost(trajectory, start_state, target_state):
    ''' Rejects trajectories that have a backwards component. '''
    s_coef, _, T = trajectory
    # if end state is before start state
    if start_state[0] > to_equation(s_coef)(T):
        return 1
    # if velocity is negative at any point
    s_vel = to_equation(differentiate(s_coef))
    dt = float(T) / 100.0
    for i in range(100):
        t = dt * i
        if s_vel(t) < 0:
            return 1

    return 0

#Proximity Costs:

def collision_cost(start_state, trajectory, lane_config, vehicles, pedestrians, objects):
    '''Penalizes collision (binary function)'''
    min_vehicle_distance = (2*VEHICLE_RADIUS)
    min_pedestrian_distance = (VEHICLE_RADIUS + Pedestrian.PEDESTRIAN_RADIUS)
    min_objects_distance = VEHICLE_RADIUS
    #split from 10-100. Higher will heavily affect performance
    ptrajectory = project_trajectory(trajectory, split = TRAJECTORY_SPLIT) 
    #check nearest
    if VEH_COLLISION:
        if vehicles:
            nearest = nearest_to_vehicles_ahead(start_state, ptrajectory, lane_config, vehicles)
            if nearest < min_vehicle_distance:
                #print("too close to vehicle")    
                return 1
    if PED_COLLISION:
        if pedestrians:
            nearest = nearest_to_pedestrians_ahead(start_state, ptrajectory, lane_config, pedestrians)
            if nearest < min_pedestrian_distance:
                #print("too close to pedestrian")    
                return 1
    if OBJ_COLLISION:
        if objects:
            nearest = nearest_to_objects_ahead(start_state, ptrajectory, lane_config, objects)
            if nearest < min_objects_distance:
                #print("too close to object (dist={})".format(nearest))    
                return 1
    return 0

def proximity_cost(start_state, trajectory, lane_config, vehicles,pedestrians,objects):
    '''Penalizes proximity to other actors'''
    min_vehicle_distance = (2*VEHICLE_RADIUS)
    min_pedestrian_distance = (VEHICLE_RADIUS + Pedestrian.PEDESTRIAN_RADIUS)
    min_object_distance = VEHICLE_RADIUS
    cost_v = cost_p = cost_o = 0
    ptrajectory = project_trajectory(trajectory, split = TRAJECTORY_SPLIT) 
    #check nearest
    if vehicles: 
        nearest = nearest_to_vehicles_ahead(start_state, ptrajectory, lane_config, vehicles)
        cost_v = logistic(min_vehicle_distance / nearest) if nearest < 10 else 0
    if pedestrians:
        nearest = nearest_to_pedestrians_ahead(start_state, ptrajectory, lane_config, pedestrians)
        cost_p = logistic(min_pedestrian_distance / nearest) if nearest < 10 else 0
    if objects:
        nearest = nearest_to_objects_ahead(start_state, ptrajectory, lane_config, objects)
        cost_o = logistic(min_object_distance / nearest) if nearest < 10 else 0
    cost = max([cost_v,cost_p,cost_o])
    #if cost > 0:
    #    print("veh {} |  ped  {} |  obj {}".format(cost_v, cost_p,cost_o))
    return cost


def nearest_to_vehicles_ahead(start_state, ptrajectory, lane_config, vehicles):
    '''Calculates the closest distance to any vehicle ahead'''
    closest = 999999
    s = start_state[0]
    d = start_state[3]
    left = lane_config.left_bound
    right = lane_config.right_bound
    for id, v in vehicles.items():
        #if close and ahead and same lane
        if ( abs(v.state.s - s) < 60) and (v.state.s > s) and (left > v.state.d > right):
            d = nearest_projected(ptrajectory,v)
            if d < closest:
                closest = d
    return closest

def nearest_to_pedestrians_ahead(start_state, ptrajectory, lane_config, pedestrians):
    '''Calculates the closest distance to any pedestrian ahead'''
    closest = 999999
    s = start_state[0]
    d = start_state[3]
    left = lane_config.left_bound
    right = lane_config.right_bound
    for id, p in pedestrians.items():
        #if close and ahead and same lane (not sure if it should only check pedestrians within lateral boundaries)
        if ( abs(p.state.s - s) < 60) and (p.state.s > s) and (left > p.state.d > right):
            #print("vehicle {} is ahead".format(k))
            d = nearest_projected(ptrajectory,p)
            if d < closest:
                closest = d
    return closest



def nearest_projected(ptrajectory, actor):
    '''Calculates the closest distance to a given actor during a trajectory.
        Relies on actor.future_state() to work
    '''
    closest = 999999
    for i in range(len(ptrajectory)):
        t = ptrajectory[i][0]
        s = ptrajectory[i][1]
        d = ptrajectory[i][4]
        targ_s, _, _, targ_d, _, _ = actor.future_state(t)
        dist = sqrt((s-targ_s)**2 + (d-targ_d)**2)
        if dist < closest:
            closest = dist
    return closest


def nearest_to_objects_ahead(start_state, ptrajectory, lane_config, objects, split = 10):
    '''Calculates the closest distance to any static objects ahead during a trajectory'''
    closest = 999999
    s = start_state[0]
    d = start_state[3]
    left = lane_config.left_bound
    right = lane_config.right_bound
    for id, o in objects.items():
        #if close and ahead and same lane
        if ( abs(o.s - s) < 100) and (o.s > s) and (left > o.d > right):
            for i in range(len(ptrajectory)):
                s = ptrajectory[i][1]
                d= ptrajectory[i][4]
                dist = sqrt( ( s - o.s )**2 + ( d - o.d)**2 )
                if dist < closest:
                    closest = dist
    return closest

def project_trajectory(trajectory,split):
    '''Project trajectory from start to goal and split in equally distributed parts.
        Higher split returns more measures, but impact performance'''
    s_coeff,d_coeff,T = trajectory    
    s_vel_coeff = differentiate(s_coeff)
    s_acc_coeff = differentiate(s_vel_coeff)
    d_vel_coeff = differentiate(d_coeff)
    d_acc_coeff = differentiate(d_vel_coeff)
    fs = to_equation(s_coeff)
    fs_vel = to_equation(s_vel_coeff)
    fs_acc = to_equation(s_acc_coeff)
    fd = to_equation(d_coeff)
    fd_vel = to_equation(d_vel_coeff)
    fd_acc = to_equation(d_acc_coeff)
    projected = []
    for i in range(split):
        t = float(i) / split * T
        s = fs(t)
        s_vel = fs_vel(t)
        s_acc = fs_acc(t) 
        d = fd(t)
        d_vel = fd_vel(t)
        d_acc = fd_acc(t) 
        projected.append([t,s,s_vel,s_acc,d,d_vel,d_acc])
    return projected

""" 

def nearest(trajectory, actor, split):
    '''Calculates the closest distance to a given actor during a trajectory.
        Relies on actor.future_state() to work
    '''
    closest = 999999
    s_,d_,T = trajectory
    s = to_equation(s_)
    d = to_equation(d_)
    for i in range(split):
        t = float(i) / split * T
        cur_s = s(t)
        cur_d = d(t)
        targ_s, _, _, targ_d, _, _ = actor.future_state(t)
        dist = sqrt((cur_s-targ_s)**2 + (cur_d-targ_d)**2)
        if dist < closest:
            closest = dist
    return closest

def nearest_to_objects_ahead(start_state, trajectory, lane_config, objects, split = 10):
    '''Calculates the closest distance to any static objects ahead during a trajectory'''
    closest = 999999
    s = start_state[0]
    d = start_state[3]
    left = lane_config.left_bound
    right = lane_config.right_bound
    for id, o in objects.items():
        #if close and ahead and same lane
        if ( abs(o.s - s) < 100) and (o.s > s) and (left > o.d > right):
            s_,d_,T = trajectory
            fs = to_equation(s_)
            fd = to_equation(d_)
            for i in range(split):
                t = float(i) / split * T
                cur_s = fs(t)
                cur_d = fd(t)
                dist = sqrt((cur_s-o.s)**2 + (cur_d-o.d)**2)
                if dist < closest:
                    closest = dist
    return closest """
