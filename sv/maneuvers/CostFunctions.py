#!/usr/bin/env python3
#rqueiroz@uwaterloo.ca
# ---------------------------------------------
# TRAJECTORY EVALUATION
# --------------------------------------------

import numpy as np
from TickSync import TickSync
from sv.maneuvers.Config import *
from util.Utils import *
from SimConfig import *
from sp.Pedestrian import *
from sv.maneuvers.FrenetTrajectory import *

#=============================== FEASIBILITY


def maneuver_feasibility(ft:FrenetTrajectory, mconfig, lane_config:LaneConfig, vehicles, pedestrians, static_objects):
    
    for c in mconfig.feasibility_constraints:
        if mconfig.feasibility_constraints[c] > 0:
            
            #direction
            if c == 'direction':
                ft.direction =  direction_cost(ft, ft.start_state)
                if ft.direction > 0:
                    ft.unfeasibility_cause +='direction,'
                    return False
                
            #collision
            elif c == 'collision':
                ft.collision = collision_cost(ft, lane_config, vehicles, pedestrians, static_objects)
                if ft.collision > 0:
                    ft.unfeasibility_cause +='collision,'
                    return False
                
            #off road
            elif c == 'off_lane':
                ft.off_lane = off_lane_cost(ft, lane_config)
                if ft.off_lane > 0:
                    ft.unfeasibility_cause +='off_lane,'
                    return False

            #acc
            elif c == 'max_long_acc':
                res, ft.max_long_acc = max_long_acc_cost(ft, mconfig.max_long_acc)
                if res > 0:
                    ft.unfeasibility_cause +='max_long_acc,'
                    return False
            elif c == 'max_lat_acc':
                res, ft.max_lat_acc = max_lat_acc_cost(ft, mconfig.max_lat_acc)
                if res > 0:
                    ft.unfeasibility_cause += 'max_lat_acc,'
                    return False

            #jerk
            elif c == 'max_lat_jerk':
                res, ft.max_lat_jerk = max_lat_jerk_cost(ft, mconfig.max_lat_jerk)
                if res > 0:
                    ft.unfeasibility_cause += 'max_lat_jerk,'
                    return False
            elif c == 'max_long_jerk':
                res, ft.max_long_jerk = max_long_jerk_cost(ft, mconfig.max_long_jerk)
                if res > 0:
                    ft.unfeasibility_cause += 'max_long_jerk,'
                    return False
            else:
                raise NotImplementedError("Feasibility function {} not implemented".format(c))
    ft.feasible = True
    return True

#=============================== MANEUVER COST

def maneuver_cost(ft:FrenetTrajectory, mconfig, lane_config:LaneConfig, vehicles, pedestrians, static_objects):
    # When a maneuver has an invalid duration, the costs cannot be computed
    if ft.T == 0 or np.isnan(ft.T):
        return
    
    for cost in mconfig.cost_weight:
        k = mconfig.cost_weight[cost]
        if k > 0:
            #basic cost
            if (cost == 'time_cost'):
                if hasattr(mconfig, 'time'):
                    ft.time_cost = k* time_cost(ft, mconfig.time.value)
                    
            elif cost == 'effic_cost':
                if hasattr(mconfig ,'vel'):
                    ft.effic_cost = k * effic_cost(ft, mconfig.time.value)
                    
            #goal
            #elif cost == 'progress_cost':
            #    ft.progress_cost = k * progress_cost(ft)
                
            elif cost == 'lane_offset_cost':
                ft.lane_offset_cost =  k * lane_offset_cost(ft, lane_config, mconfig.expected_offset_per_sec)
                
            #jerk
            elif cost == 'total_lat_jerk_cost':
                ft.total_lat_jerk_cost = k * total_lat_jerk_cost(ft, mconfig.expected_lat_jerk_per_sec)
                
            elif cost == 'total_long_jerk_cost':
                ft.total_long_jerk_cost = k *total_long_jerk_cost(ft, mconfig.expected_long_jerk_per_sec)
                
            #acc
            elif cost == 'total_long_acc_cost':
                ft.total_long_acc_cost = k * total_long_acc_cost(ft, mconfig.expected_long_acc_per_sec)
                
            elif cost == 'total_lat_acc_cost':
                ft.total_lat_acc_cost = k * total_lat_acc_cost(ft, mconfig.expected_lat_acc_per_sec)
                
            #proximity
            elif cost == 'proximity_cost':
                ft.proximity_cost = k * proximity_cost(ft, lane_config, vehicles, pedestrians, static_objects )
                
            else:
                raise NotImplementedError("Cost function {} not implemented".format(cost))
    
    ft.get_total_cost()
    #return sum(C)
    

#=============================== INDIVIDUAL COST FUNCTIONS


#Efficiency Cost:

def effic_cost(frenet_traj:FrenetTrajectory,target_vel):
    '''Penalizes low average velocity'''
    total = 0
    ptrajectory = frenet_traj.projected_trajectory

    for i in range(len(ptrajectory)):
        total += ptrajectory[i][1][1] #1=s_state, 1=s_vel
    
    avg_vel = float(total) / len(ptrajectory)

    if avg_vel == 0:
        return 0

    return logistic( 2*(target_vel-avg_vel) / avg_vel)

def time_cost(frenet_traj:FrenetTrajectory, target_t):
    '''Penalizes trajectories longer or shorter than target time.'''
    diff =  float(abs(frenet_traj.T-target_t))
    return logistic( diff / target_t)

'''
#deprecated
#This cost is not compatible with current architecture
def progress_cost(frenet_traj:FrenetTrajectory):
    #Penalizes trajectories that end away from the target state.
    
    end_state = frenet_traj.get_state_at(frenet_traj.T)
    target_s =  frenet_traj.target_state[0]
    #if quartic polyn, s=0.0 means any s is accepted
    if target_s == 0.0: 
        target_s = end_state[0]
    
    end_arr = np.array([
        end_state[0],
        end_state[3]
    ])
    target_arr = np.array([
        target_s,
        frenet_traj.target_state[3]
    ])
    diff = end_arr - target_arr
    if diff[0] < 0:
        # harshly penalize overshooting trajectories
        cost = 1
    else:
        rel_error = np.linalg.norm(diff) / np.linalg.norm(target_arr)
        cost = logistic(rel_error)
    print("target {}, actual {}, diff {} cost {}".format(
            target_arr,
            end_arr,
            diff,
            cost
        ))
    return cost
'''

#Road geometry Cost:

def lane_offset_cost(frenet_traj:FrenetTrajectory,lane_config:LaneConfig, expected_offset_per_sec):
    '''Penalizes distance from lane center during the entire trajectory'''
    total_offset = 0
    ptrajectory = frenet_traj.projected_trajectory
    T = frenet_traj.T
    dt = T / len(ptrajectory)
    for i in range(len(ptrajectory)):
        d = ptrajectory[i][2][0] #2=d_state, 0=d
        offset = d - lane_config.get_central_d()
        total_offset += abs(offset)*dt
    offset_per_second = total_offset / T
    #print("total offset is per second {:.2f} ".format(offset_per_second))
    cost = logistic(offset_per_second/expected_offset_per_sec)
    return cost

    '''
    #_, d_coef, T = trajectory
    #d_eq = to_equation(d_coef)
    #target_d = d_eq(T)
    central_d = (lane_config.left_bound - lane_config.right_bound)/2 + lane_config.right_bound
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
    '''

def off_lane_cost(frenet_traj:FrenetTrajectory,lane_config:LaneConfig):
    '''
    Penalizes trajectories going beyond lane boundaries (binary function)
    Current implementation relies on a fixed lane width.
    #TODO: Adapt lane config to store the lane width over a distance ahead
    '''
    ptrajectory = frenet_traj.projected_trajectory
    for i in range(len(ptrajectory)):
        d = ptrajectory[i][2][0] #2=d_state, 0=d
        if not (lane_config.left_bound > d > lane_config.right_bound):
            #print("off_lane")
            return 1
    return 0
    '''
    #old Implementation:
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
    '''


#Jerk Cost:

def max_long_jerk_cost(frenet_traj:FrenetTrajectory, expected_max_jerk):
    '''Penalizes high longitudinal jerk (binary function)'''
    ptrajectory = frenet_traj.projected_trajectory
    all_jerks = [ptrajectory[i][1][3] for i in range(len(ptrajectory))] #1=s_state, 3=s_jerk
    max_jerk = max(all_jerks, key=abs)
    if abs(max_jerk) > expected_max_jerk:
        return 1 , max_jerk
    else:
        return 0 , max_jerk
    '''
    #old Implementation:
    s_coef, _, T = trajectory
    jerk = to_equation(differentiate(differentiate(differentiate(s_coef))))
    all_jerks = [jerk(float(T)/100 * i) for i in range(100)]
    max_jerk = max(all_jerks, key=abs)
    if abs(max_jerk) > expected_max_jerk:
        return 1
    else:
        return 0
    '''

def max_lat_jerk_cost(frenet_traj:FrenetTrajectory, expected_max_jerk):
    '''Penalizes high lateral jerk (binary function)'''
    ptrajectory = frenet_traj.projected_trajectory
    all_jerks = [ptrajectory[i][2][3] for i in range(len(ptrajectory))] #2=d_state, 3=d_jerk
    max_jerk = max(all_jerks, key=abs)
    if abs(max_jerk) > expected_max_jerk:
        return 1 , max_jerk
    else:
        return 0 , max_jerk
    ''' 
    #old Implementation:
    _, d_coef, T = trajectory
    jerk = to_equation(differentiate(differentiate(differentiate(d_coef))))
    all_jerks = [jerk(float(T)/100 * i) for i in range(100)]
    max_jerk = max(all_jerks, key=abs)
    if abs(max_jerk) > expected_max_jerk:
        return 1
    else:
        return 0
    '''

def total_long_jerk_cost(frenet_traj:FrenetTrajectory, expected_jerk_per_sec):
    '''Penalizes high longitudinal jerk '''
    #s_coef, _, T = trajectory
    #jerk = to_equation(differentiate(differentiate(differentiate(s_coef))))
    T = frenet_traj.T
    ptrajectory = frenet_traj.projected_trajectory
    dt = T / len(ptrajectory)
    all_jerk = [ abs( ptrajectory[i][1][3] * dt ) for i in range(len(ptrajectory))] #1-s_state 3-d_jerk
    acc_per_sec = sum(all_jerk) / T
    return logistic(acc_per_sec / expected_jerk_per_sec )
    '''
    T = frenet_traj.T
    total_jerk = 0
    dt = float(T) / 100.0
    for i in range(100):
        t = dt * i
        #j = jerk(t)
        j = frenet_traj.fs_jerk(t)
        total_jerk += abs(j*dt)
    jerk_per_second = total_jerk / T
    return logistic(jerk_per_second / expected_jerk_per_sec )
    '''

def total_lat_jerk_cost(frenet_traj:FrenetTrajectory, expected_jerk_per_sec):
    '''Penalizes high lateral jerk '''
    #_, d_coef, T = trajectory
    #jerk = to_equation(differentiate(differentiate(differentiate(d_coef))))
    T = frenet_traj.T
    ptrajectory = frenet_traj.projected_trajectory
    dt = T / len(ptrajectory)
    all_jerk = [ abs( ptrajectory[i][2][3] * dt ) for i in range(len(ptrajectory))] #2-d_state 3-d_jerk
    acc_per_sec = sum(all_jerk) / T
    return logistic(acc_per_sec / expected_jerk_per_sec )

    '''
    T = frenet_traj.T
    total_jerk = 0
    dt = float(T) / 100.0
    for i in range(100):
        t = dt * i
        #j = jerk(t)
        j = frenet_traj.fd_jerk(t)
        total_jerk += abs(j*dt)
    jerk_per_second = total_jerk / T
    return logistic(jerk_per_second / expected_jerk_per_sec)
    '''

#Acc
def max_long_acc_cost(frenet_traj:FrenetTrajectory, expected_max_acc):
    '''Penalizes high longitudinal acceleration (binary function)'''
    #s_coef, _, T = trajectory
    #return max_acc_cost(s_coef, T, expected_max_acc)
    ptrajectory = frenet_traj.projected_trajectory
    all_acc = [ptrajectory[i][1][2] for i in range(len(ptrajectory))] #1=s_state, 2=s_acc
    max_acc = max(all_acc, key=abs)
    if abs(max_acc) > expected_max_acc:
        return 1 , max_acc
    else:
        return 0 , max_acc

def max_lat_acc_cost(frenet_traj:FrenetTrajectory, expected_max_acc):
    '''Penalizes high lateral acceleration (binary function)'''
    #_, d_coef, T = trajectory
    #return max_acc_cost(d_coef, T, expected_max_acc)
    ptrajectory = frenet_traj.projected_trajectory
    all_acc = [ptrajectory[i][2][2] for i in range(len(ptrajectory))] #2=d_state, 2=s_acc
    max_acc = max(all_acc, key=abs)
    if abs(max_acc) > expected_max_acc:
        return 1 , max_acc
    else:
        return 0 , max_acc

'''def max_acc_cost(x_coef, T, expected_max_acc):
    acc = to_equation(differentiate(differentiate(x_coef)))
    all_accs = []
    dt = float(T) / 100.0
    for i in range(100):
        t = dt * i
        all_accs.append(acc(t))
    max_acc = max(all_accs, key=abs)
    if abs(max_acc) > expected_max_acc:
        return 1, max_acc
    else:
        return 0, max_acc
'''

def total_long_acc_cost(frenet_traj:FrenetTrajectory, expected_long_acc_per_sec):
    '''Penalizes high longitudinal acceleration'''
    #s_coef, _, T = trajectory
    #return total_acc_cost(s_coef, T, expected_long_acc_per_sec)
    T = frenet_traj.T
    ptrajectory = frenet_traj.projected_trajectory
    dt = T / len(ptrajectory)
    all_acc = [ abs( ptrajectory[i][1][2] * dt ) for i in range(len(ptrajectory))] #1-sstate 2-s_acc
    acc_per_sec = sum(all_acc) / T
    return logistic(acc_per_sec / expected_long_acc_per_sec )
    '''
    T = frenet_traj.T
    total_acc = 0
    dt = float(T) / 100.0
    for i in range(100):
        t = dt * i
        a = frenet_traj.fs_acc(t)
        total_acc += abs(a*dt)
    cc_per_sec = total_acc / T
    return logistic(acc_per_sec / expected_long_acc_per_sec )
    '''

def total_lat_acc_cost(frenet_traj:FrenetTrajectory, expected_lat_acc_per_sec):
    '''Penalizes high lateral acceleration'''
    #_, d_coef, T = trajectory
    #return total_acc_cost(d_coef, T, expected_lat_acc_per_sec)
    T = frenet_traj.T
    ptrajectory = frenet_traj.projected_trajectory
    dt = T / len(ptrajectory)
    all_acc = [ abs( ptrajectory[i][2][2] * dt ) for i in range(len(ptrajectory))] #2-d_state 2-d_acc
    acc_per_sec = sum(all_acc) / T
    return logistic(acc_per_sec / expected_lat_acc_per_sec )
    '''
    T = frenet_traj.T
    total_acc = 0
    dt = float(T) / 100.0
    for i in range(100):
        t = dt * i
        a = frenet_traj.fd_acc(t)
        total_acc += abs(a*dt)
    acc_per_sec = total_acc / T
    return logistic(acc_per_sec / expected_lat_acc_per_sec )
    '''

'''
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
'''

def direction_cost(frenet_traj:FrenetTrajectory, start_state):
    ''' Rejects trajectories that have a backwards component. '''
    T = frenet_traj.T
    vel_threshold =  -0.0001 #some trajectories have a small inversion at the beginning
    # if end state is before start state
    if start_state[0] > frenet_traj.fs(T):
        return 1
    # if velocity is negative at any point
    dt = float(T) / 99
    for i in range(100):
        t = dt * i
        if frenet_traj.fs_vel(t) < vel_threshold:
            return 1
    return 0

    '''
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
    '''

#Proximity Costs:

def collision_cost(frenet_traj:FrenetTrajectory, lane_config, vehicles, pedestrians, objects):
    '''Penalizes collision (binary function)'''
    min_vehicle_distance = (2*VEHICLE_RADIUS)
    min_pedestrian_distance = (VEHICLE_RADIUS + PEDESTRIAN_RADIUS)
    min_objects_distance = VEHICLE_RADIUS
    ptrajectory = frenet_traj.projected_trajectory
    #check nearest
    if vehicles:
        nearest = nearest_to_vehicles_ahead(frenet_traj.start_state, ptrajectory, lane_config, vehicles)
        if nearest < min_vehicle_distance:
            #print("too close to vehicle")    
            return 1
    
    if pedestrians:
        nearest = nearest_to_pedestrians_ahead(frenet_traj.start_state, ptrajectory, lane_config, pedestrians)
        if nearest < min_pedestrian_distance:
            #print("too close to pedestrian")    
            return 1
    
    if objects:
        nearest = nearest_to_objects_ahead(frenet_traj.start_state, ptrajectory, lane_config, objects)
        if nearest < min_objects_distance:
            #print("too close to object (dist={})".format(nearest))    
            return 1
    return 0

def proximity_cost(frenet_traj:FrenetTrajectory, lane_config, vehicles,pedestrians,objects):
    '''Penalizes proximity to other actors'''
    min_vehicle_distance = (2*VEHICLE_RADIUS)
    min_pedestrian_distance = (VEHICLE_RADIUS + PEDESTRIAN_RADIUS)
    min_object_distance = VEHICLE_RADIUS
    cost_v = cost_p = cost_o = 0
    ptrajectory = frenet_traj.projected_trajectory
    #check nearest
    if vehicles: 
        nearest = nearest_to_vehicles_ahead(frenet_traj.start_state, ptrajectory, lane_config, vehicles)
        cost_v = logistic(min_vehicle_distance / nearest) if nearest < 10 else 0
    if pedestrians:
        nearest = nearest_to_pedestrians_ahead(frenet_traj.start_state, ptrajectory, lane_config, pedestrians)
        cost_p = logistic(min_pedestrian_distance / nearest) if nearest < 10 else 0
    if objects:
        nearest = nearest_to_objects_ahead(frenet_traj.start_state, ptrajectory, lane_config, objects)
        cost_o = logistic(min_object_distance / nearest) if nearest < 10 else 0
    cost  = sum([cost_v,cost_p,cost_o])
    #if cost > 0:
    #    print("veh {} |  ped  {} |  obj {}".format(cost_v, cost_p,cost_o))
    return cost


def nearest_to_vehicles_ahead(start_state, ptrajectory, lane_config, vehicles):
    '''Calculates the closest distance to any vehicle ahead'''
    closest = 999999
    s = start_state[0]
    d = start_state[3]

    for id, v in vehicles.items():
        #if close and ahead and same lane
        width = v.width
        left = lane_config.left_bound + width
        right = lane_config.right_bound - width
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
        s = ptrajectory[i][1][0] #1=s_state, 0=s
        d = ptrajectory[i][2][0] #2=d_state, 0=d
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
                s = ptrajectory[i][1][0] #1=s_state, 0=s
                d = ptrajectory[i][2][0] #2=d_state, 0=d
                dist = sqrt( ( s - o.s )**2 + ( d - o.d)**2 )
                if dist < closest:
                    closest = dist
    return closest

