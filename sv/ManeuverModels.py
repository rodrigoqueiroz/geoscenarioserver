#!/usr/bin/env python3
#rqueiroz@uwaterloo.ca
# --------------------------------------------
# GEOSCENARIO Micro Maneuver Models for Motion Planning
# --------------------------------------------
from sv.SDVTrafficState import *
import numpy as np
from copy import copy
import itertools
from TickSync import *
from numpy.core.arrayprint import _none_or_positive_arg
from numpy.core.records import array
import logging
log = logging.getLogger(__name__)
#from multiprocessing import Pool as ThreadPool
from sv.CostFunctions import maneuver_feasibility, maneuver_cost
from sv.ManeuverConfig import *
from SimConfig import *
from Actor import *
from sv.FrenetTrajectory import *
from typing import Callable
from sv.ManeuverUtils import *


def plan_maneuver(sdv, mconfig, traffic_state):
    #log.info('MANEUVER {}:  Vehicle {}'.format(mconfig.mkey,vid))

    vid = int(sdv.id)
    #Micro maneuver layer
    if (mconfig.mkey == Maneuver.M_VELKEEP):
        return plan_velocity_keeping(vid, mconfig, traffic_state)
    elif (mconfig.mkey == Maneuver.M_REVERSE):
        return plan_reversing(vid, mconfig, traffic_state)
    elif (mconfig.mkey == Maneuver.M_STOP):
        return plan_stop(sdv, mconfig, traffic_state)
    elif (mconfig.mkey == Maneuver.M_FOLLOW):
        return plan_following(vid, mconfig, traffic_state)
    elif (mconfig.mkey == Maneuver.M_LANESWERVE):
        return plan_laneswerve(vid, mconfig, traffic_state)
    elif (mconfig.mkey == Maneuver.M_CUTIN):
        return plan_cutin(sdv, mconfig, traffic_state)
    
    log.error(f"Vehicle {vid} trying to execute maneuver not implemented {mconfig.mkey}")
    return None, None
    

def plan_velocity_keeping(vid, mconfig:MVelKeepConfig, traffic_state:TrafficState):
    """
    VELOCITY KEEPING
    Driving with no vehicle directly ahead
    No target point, but needs to adapt to a desired velocity
    """
    vehicle_state:VehicleState = traffic_state.vehicle_state
    lane_config:LaneConfig = traffic_state.lane_config
    
    #cap target vel to maximum difference
    #this will smooth the trajectory when starting
    if (mconfig.vel.value - vehicle_state.s_vel) > mconfig.max_diff:
        target_vel = copy(mconfig.vel)
        target_vel.value = vehicle_state.s_vel + mconfig.max_diff
    else:
        target_vel =  mconfig.vel

    #generate alternative targets:
    target_state_set = []
    for t in mconfig.time.get_samples():
        #longitudinal movement: goal is to reach velocity
        for vel in target_vel.get_samples():
            s_target = [0,vel,0] #pos not relevant
            #lateral movement
            for di in mconfig.lat_target.get_samples(lane_config):
                d_target = [di,0,0]
                #add target
                target_state_set.append((s_target,d_target,t))

    

    best, candidates = optimized_trajectory(vid, mconfig, traffic_state, target_state_set, s_solver=quartic_polynomial_solver)
    #log.info('PLAN VK {} target vel {}'.format(vid,target_vel))
    #log.info(best)
    return best, candidates 

def plan_reversing(vid, mconfig:MReverseConfig, traffic_state:TrafficState):
    """
    REVERSING
    Driving in reverse
    No target point, but needs to adapt to a desired velocity
    """
    lane_config:LaneConfig = traffic_state.lane_config

    #generate alternative targets:
    target_state_set = []
    for t in mconfig.time.get_samples():
        #longitudinal movement: goal is to reach velocity
        for vel in mconfig.vel.get_samples():
            s_target = [0,-vel,0] #pos not relevant
            #lateral movement
            for di in mconfig.lat_target.get_samples(lane_config):
                d_target = [di,0,0]
                #add target
                target_state_set.append((s_target,d_target,t))

    best, candidates = optimized_trajectory(vid, mconfig, traffic_state, target_state_set, s_solver=quartic_polynomial_solver)
    return best, candidates 

def plan_following(vid, mconfig:MFollowConfig, traffic_state:TrafficState):
    """
    VEHICLE FOLLOWING
    Moving target point, requiring a certain temporal safety distance to the vehicle ahead (constant time gap law).
    Predict leading vehicle (assume constant acceleration)
    """
    #log.info('Maneuver:  Vehicle {} Following {}'.format(vid,mconfig.target_vid))
    vehicle_state:VehicleState = traffic_state.vehicle_state
    lane_config:LaneConfig = traffic_state.lane_config
    vehicles = traffic_state.traffic_vehicles
    
    s_start = vehicle_state.get_S()
    d_start = vehicle_state.get_D()
    target_vid = mconfig.target_vid
    time_gap = mconfig.time_gap
    
    #Pre conditions. Can the vehicle be followed?
    if target_vid is None:
        return None, None
    leading_vehicle = vehicles[target_vid]
    #Is target ahead?
    if (s_start[0] >= leading_vehicle.state.s):
        return None, None
    #Is target on the same lane?
    if lane_config.get_current_lane(d_start[0]).id != lane_config.get_current_lane(leading_vehicle.state.d).id:
        log.warning("Leading vehicle {} is on a different lane.".format(target_vid))
        return None, None

    # check if we need to deccel to increase gap, for the case when both vehicles
    # have positive velocity.
    #dist_between_vehicles = leading_vehicle.state.s - VEHICLE_RADIUS - s_start[0] - VEHICLE_RADIUS - VEHICLE_RADIUS
    #ttc = dist_between_vehicles / abs(s_start[1]) if s_start[1] != 0 else float('inf')
    #following_too_close = ttc < mconfig.time_gap

    #generate alternative targets:
    target_state_set = []
   
    for t in mconfig.time.get_samples():
        #longitudinal movement: goal is to keep safe distance from leading_vehicle
        s_target = [0,0,0]
        # If leading vehicle is slower than some threshold velocity, our goal is to stop.
        if abs(leading_vehicle.state.s_vel) < 1.5:
            #log.info("lead stopped")
            #s_target[0] = leading_vehicle.state.s - 5 - VEHICLE_RADIUS * 2  #stop some meters behind stopped vehicle
            s_target[0] = leading_vehicle.state.s - leading_vehicle.length - mconfig.stop_distance  #some meters behind stopped vehicle
            d_target = [d_start[0],0,0]                     #keep in same lateral position
            target_state_set.append((s_target,d_target,t))  #add target
        else:
            # match leading vehicle speed
            s_lv = leading_vehicle.future_state(t)[:3]
            #log.info("Matching lead. State {} to {} at time {}".format(leading_vehicle.state.get_S(),s_lv,t))
            s_target[0] = s_lv[0] - (time_gap * s_lv[1])
            s_target[1] = s_lv[1]
            s_target[2] = s_lv[2]
            #lateral movement
            for di in mconfig.lat_target.get_samples(lane_config):
                d_target = [di,0,0]
                #add target
                target_state_set.append((s_target,d_target,t))
   
    best, candidates = optimized_trajectory(vid, mconfig, traffic_state, target_state_set, s_solver=quintic_polynomial_solver)
    # if best:
    #     log.info("starting FOLLOW: {:.3f} {:.3f} {:.3f}".format(start_state[0], start_state[1], start_state[2]))
    #     log.info("targetting end {:.3f} {:.3f} {:.3f} at t={:.3f}".format(best_target[0][0], best_target[0][1], best_target[0][2], best_target[-1]))
    # else:
    #     log.info("No FOLLOW traj")
    return best, candidates


def plan_laneswerve(vid, mconfig:MLaneSwerveConfig, traffic_state:TrafficState):
    """
    LANE CHANGE SWERVE
    Swerve maneuver to another lane
    No vehicles affecting the lane change, except in case of collision detection (if Collision is on).
    """
    vehicle_state:VehicleState = traffic_state.vehicle_state
    s_start = vehicle_state.get_S()
    lane_config:LaneConfig = traffic_state.lane_config
    target_lid = mconfig.target_lid

    #Find target lane
    target_lane_config = None
    if (lane_config.id == target_lid):
        target_lane_config = lane_config
        log.warning('already in target lane {}'.format(target_lid))
        return None, None
    else:
        target_lane_config = lane_config.get_neighbour(target_lid)
    if not target_lane_config:
        log.warning('target lane {} not found, is it a neighbour lane?'.format(target_lid))
        return None, None
    
    #generates alternative targets:
    target_state_set = []
    for t in mconfig.time.get_samples():
        #longitudinal movement: goal is to keep velocity
        vel = s_start[1]
        s_target = [0,vel,0] #pos not relevant, no acc expected at the end
        #lateral movement
        #for di in target_lane_config.get_samples():
        for di in mconfig.lat_target.get_samples(target_lane_config):
            d_target = [di,0,0] #no lateral movement expected at the end
            target_state_set.append((s_target,d_target,t))

    best, candidates = optimized_trajectory(vid, mconfig, traffic_state, target_state_set, s_solver=quartic_polynomial_solver)
    return best, candidates 

def plan_cutin(sdv, mconfig:MCutInConfig, traffic_state:TrafficState):
    """
    CUT-IN LANE SWERVE
    """
    vid = int(sdv.id)
    lane_config:LaneConfig = traffic_state.lane_config
    vehicles = traffic_state.traffic_vehicles
    target_id = mconfig.target_vid
    delta = mconfig.delta_s + mconfig.delta_d
    delt_s_sampling = mconfig.delta_s_sampling

    if (target_id not in vehicles):
        log.warning("Target vehicle {} is not in traffic".format(target_id))
        return None, None

    target_lane_config = lane_config.get_current_lane(vehicles[target_id].state.d)
    if not target_lane_config:
        log.warning("Target vehicle {} is not in an adjacent lane".format(target_id))
        return None, None
    elif target_lane_config.id == lane_config.id:
        log.warning("Already in target lane")
        return None, None

    # List[(target s, target d, t)]
    target_state_set = []
    for t in mconfig.time.get_samples():
        #main goal is relative to target vehicle predicted final position
        state_relative_to = vehicles[target_id].future_state(t)
        #log.info("cuttee future state at t={}: {:.3f} {:.3f} {:.3f}".format(t, state_relative_to[0], state_relative_to[1], state_relative_to[2]))
        
        #Bugfix: sampling should be in the delta, not the final S position
        delta_s_pos =        MP(delta[0], delt_s_sampling[0][0], delt_s_sampling[0][1])
        delta_s_vel =    MP(delta[1], delt_s_sampling[1][0], delt_s_sampling[1][1])
        delta_s_acc =    MP(delta[2], delt_s_sampling[2][0], delt_s_sampling[1][1])
        
        #+= 2 * VEHICLE_RADIUS
        state_relative_to[0] += sdv.length*2

        dts_samples = delta_s_pos.get_samples()
        dts_vel_samples = delta_s_vel.get_samples()
        dts_acc_samples = delta_s_acc.get_samples()

        for dts in dts_samples:
            for dts_vel in dts_vel_samples:
                for dts_acc in dts_acc_samples:
                    #log.info("with delta: {:.3f} {:.3f} {:.3f}".format(dts, dts_vel, dts_acc))
                    goal_state_relative = np.array(state_relative_to[0:3]) + np.array([dts,dts_vel,dts_acc])
                    s = goal_state_relative[0]
                    s_vel = goal_state_relative[1]
                    s_acc = goal_state_relative[2]
                    s_target = np.array([s, s_vel, s_acc])
                    for di in mconfig.lat_target.get_samples(target_lane_config):
                        d_target = [di, 0, 0] # no lateral movement expected at the end
                        target_state_set.append((s_target, d_target, t))

        #instead of:
        #goal_state_relative = np.array(state_relative_to) + np.array(delta)
        #goal_state_relative[0] += VEHICLE_LENGTH
        #log.info("with delta: {:.3f} {:.3f} {:.3f}".format(goal_state_relative[0], goal_state_relative[1], goal_state_relative[2]))
        
        # sample in s
        #s_samples = []
        #for i in range(3):
        #    ds = (mconfig.delta_s_sampling[i][0] / 100) * goal_state_relative[i]
        #    s_samples.append(
        #        np.linspace(goal_state_relative[i] - ds, goal_state_relative[i] + ds, num=mconfig.delta_s_sampling[i][1])
        #    )
        #log.info("s samples {}".format(s_samples))
        #for s in s_samples[0]:
        #    for s_vel in s_samples[1]:
        #        for s_acc in s_samples[2]:
        #            s_target = np.array([s, s_vel, s_acc])
        #            for di in mconfig.lat_target.get_samples(target_lane_config):
        #                # no lateral movement expected at the end
        #                d_target = [di, 0, 0]
        #                target_state_set.append((s_target, d_target, t))

    
    best:FrenetTrajectory = None
    #log.info("targets: {}".format(["({:.3f} {:.3f} {:.3f})\n".format(t[0][0], t[0][1], t[0][2]) for t in target_state_set]))
    best, candidates = optimized_trajectory(vid, mconfig, traffic_state, target_state_set, s_solver=quintic_polynomial_solver)
    
    #stat
    #if best:
    #    print(best.target_state)
    #else:
    #    causes = []
    #    for traj in candidates:
    #        causes.append(traj.unfeasibility_cause)
    #    print(list(set(causes)))
    #print(best.fs(best.T))
    #a = best.target_state[0]
    #b = vehicles[target_id].future_state(best.T)[0]
    #c = traffic_state.vehicle_state.s #should be zero
    #d = vehicles[target_id].state.s #
    #print ("CTI: v_s{}, tarv_s{}, diff{}, tar_delta{}, tar_s {}, tarv_fut_s {}, diff {}".format(
    #        c,      d,      (c-d),      delta,      a,          b,          (a-b-VEHICLE_LENGTH)))

    return best, candidates


def plan_stop(sdv, mconfig:MStopConfig, traffic_state:TrafficState):
    """
    STOP
    Stop can be a stop request by time and/or distance from current pos.
    Or optionally have a specific target position to stop (stop line, before an object, etc).
    """
    vid = int(sdv.id)
    vehicle_state:VehicleState = traffic_state.vehicle_state
    lane_config:LaneConfig = traffic_state.lane_config
    vehicles = traffic_state.traffic_vehicles
    
    #start
    s_start = vehicle_state.get_S()
    d_start = vehicle_state.get_D()

    # Find s position for dynamic target 
    if mconfig.target == MStopConfig.StopTarget.GOAL:
        mconfig.pos = traffic_state.goal_point_frenet[0]
    elif mconfig.target == MStopConfig.StopTarget.STOP_LINE:
       mconfig.pos = lane_config.stopline_pos
       for intersection in traffic_state.intersections:
            mconfig.pos = intersection.stop_position[0]
            break
    #log.info("PLAN STOP: can not find stop position in intersection")            
    
    #adjust target pos to vehicle length
    target_pos = mconfig.pos - sdv.length/2 - mconfig.distance

    #Already stopped?
    if (abs(s_start[1]) <= 0.05):
        #log.warning('Vehicle already stopped')
        #TODO: Need another stop maneuver (yielding) for proper configuration
        #if already stopped and not at stopping point, move to it
        if target_pos > 1:
            log.info("PLAN STOP: move to adjust target pos {}".format(target_pos))
            return plan_velocity_keeping(vid, MVelKeepConfig(), traffic_state)
        ft = FrenetTrajectory()
        return ft, None
    
    #log.info('PLAN STOP at pos {} target {}'.format(mconfig.pos, mconfig.target))

    #adjust target pos to possible dynamic elements:
    lv = get_leading_vehicle(vehicle_state,lane_config,vehicles)
    if lv:
        #max_pos = lv.state.s - VEHICLE_RADIUS*3
        max_pos = lv.state.s - sdv.length - (max(mconfig.distance,2)) #either use configured distance or a minimum of 2 behind another vehicle
        if target_pos > max_pos:
            #log.warning('Vehicle {} stop target {} adjusted to lead pos {}. New target {}'.format(vid,target_pos, lv.state.s, max_pos))
            target_pos = max_pos
    
    stop_distance = target_pos - s_start[0]
    expected_time = 2*stop_distance / (vehicle_state.s_vel) #assuming uniform acceleration
    target_time = MP(expected_time,40,6) #bound >40% recommended for safely finding a suitable stop time
    
    # within a certain distance generating new trajectory doesn't make sense
    if target_pos < 1: 
        #or abs(target_pos - vehicle_state.s) < 1:
        log.warning('PLAN STOP Vehicle {} target position {} is too close or behind'.format(vid,target_pos))
        #mconfig.type = MStopConfig.Type.NOW
        #s_solver = quartic_polynomial_solver
        return None, None

    # when vehicle is past the target point, switch to STOP NOW
    #if (target_pos - vehicle_state.s) < 0:
    #    log.warning('Vehicle {} stop target position behind. diff={}'.format(vid,target_pos - vehicle_state.s))
    #    #mconfig.type = MStopConfig.Type.NOW    
    #    #s_solver = quartic_polynomial_solver
    #    return None, None
    #else:

    s_solver = quintic_polynomial_solver

    #targets
    target_state_set = []
    #generates alternative targets
    for t in target_time.get_samples():
        #longitudinal movement: goal is to reach vel and acc 0
        s_target = [target_pos, 0, 0]
        #lateral movement
        for di in mconfig.lat_target.get_samples(lane_config):
            d_target = [di, 0, 0]
            #add target
            target_state_set.append((s_target, d_target, t))
    
    best, candidates = optimized_trajectory(vid, mconfig, traffic_state, target_state_set, s_solver = s_solver)
    return best, candidates 


#===TRAJECTORY OPTIMIZATION ===

def optimized_trajectory(vid:int, mconfig:MConfig, traffic_state:TrafficState, target_state_set, s_solver):
    """
    Generates and select the best trajectory for the maneuver.
    Returns the resulting trajectory and a list of candidates for debug purposes.
    """
    vehicle_state:VehicleState = traffic_state.vehicle_state
    start_state = vehicle_state.get_S() + vehicle_state.get_D()
    lane_config:LaneConfig = traffic_state.lane_config
    vehicles = traffic_state.traffic_vehicles
    pedestrians = traffic_state.pedestrians
    static_objects = traffic_state.static_objects
    
    #find trajectories
    trajectories = []
    traj_solver = lambda x: find_trajectory(x, s_solver)
    trajectories = list(map(traj_solver, zip(itertools.repeat(start_state), target_state_set)))
    
    #evaluate feasibility
    #New: holding a data object for trajectories to hold and share partial costs across multiple cost functions 
    frenet_trajectories = []
    feasible = []
    for traj, target_state in zip(trajectories, target_state_set):
        ft = FrenetTrajectory()
        ft.set_trajectory(traj[0],traj[1],target_state[2],mconfig.cost_precision)
        ft.start_state = start_state
        ft.target_state = np.concatenate([target_state[0],target_state[1]])
        frenet_trajectories.append(ft)
        if maneuver_feasibility(ft, mconfig, lane_config, vehicles, pedestrians, static_objects):
            ft.feasible = True
            feasible.append(ft)
    
    for ft in feasible:
        maneuver_cost(ft, mconfig, lane_config, vehicles, pedestrians, static_objects)
    
    if len(feasible) == 0:
        #log.warning("No feasible trajectory to select from state {}".format(start_state))
        #for traj in frenet_trajectories:
        #    log.warning(traj.unfeasibility_cause)
        return None, frenet_trajectories
    
    #select best by total cost
    best_ft:FrenetTrajectory = min(feasible, key=lambda traj:traj.get_total_cost())

    #returning best, and all candidates for debug
    return best_ft, frenet_trajectories


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
    # append 0 as the 6th coefficient
    alphas = np.concatenate([np.array([a_0, a_1, a_2]), a_3_4, [0]])
    return alphas

def find_trajectory(traj_bounds,
                    s_solver:Callable = quintic_polynomial_solver,
                    d_solver:Callable = quintic_polynomial_solver):
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
    s_coef = s_solver(s_start, s_target, t)
    d_coef = d_solver(d_start, d_target, t)
    return tuple([s_coef, d_coef, t])
