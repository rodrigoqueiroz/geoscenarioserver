#!/usr/bin/env python
#rqueiroz@uwaterloo.ca
# --------------------------------------------
# GEOSCENARIO Micro Maneuver Models for Motion Planning
# --------------------------------------------
import numpy as np
from copy import copy
import itertools
import glog as log
#from multiprocessing import Pool as ThreadPool
from sv.CostFunctions import maneuver_feasibility, maneuver_cost
from sv.ManeuverConfig import *
from SimConfig import *
from Actor import *
from typing import Callable
from sv.ManeuverUtils import *


def plan_maneuver(man_key, mconfig, vehicle_state, lane_config, vehicles, pedestrians, static_objects):
    #Micro maneuver layer
    if (man_key == Maneuver.M_VELKEEP):
        planner = plan_velocity_keeping
    elif (man_key == Maneuver.M_REVERSE):
        planner = plan_reversing
    elif (man_key == Maneuver.M_STOP):
        planner = plan_stop
    elif (man_key == Maneuver.M_FOLLOW):
        planner = plan_following
    elif (man_key == Maneuver.M_LANESWERVE):
        planner = plan_laneswerve
    elif (man_key == Maneuver.M_CUTIN):
        planner = plan_cutin

    return planner(vehicle_state, mconfig, lane_config, vehicles, pedestrians, static_objects)
    

def plan_velocity_keeping(vehicle_state:VehicleState, mconfig:MVelKeepConfig, lane_config:LaneConfig, vehicles=None,  pedestrians = None, static_objects=None):
    """
    VELOCITY KEEPING
    Driving with no vehicle directly ahead
    No target point, but needs to adapt to a desired velocity
    """
    #print ('Maneuver: Velocity Keeping')
    s_start = vehicle_state.get_S()
    d_start = vehicle_state.get_D()
    
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
            for di in lane_config.get_samples():
                d_target = [di,0,0]
                #add target
                target_state_set.append((s_target,d_target,t))

    #print ('Targets: {}'.format(len(target_state_set)))

    best, best_target, candidates, unfeasible = optimized_trajectory(
        vehicle_state, target_state_set, mconfig, lane_config, vehicles, pedestrians, static_objects, 
        s_solver=quartic_polynomial_solver)
    return best, candidates, unfeasible

def plan_reversing(vehicle_state:VehicleState, mconfig:MReverseConfig, lane_config:LaneConfig, vehicles=None, pedestrians = None, static_objects=None):
    """
    REVERSING
    Driving in reverse
    No target point, but needs to adapt to a desired velocity
    """
    #print ('Maneuver: Reverse')
    s_start = vehicle_state.get_S()
    d_start = vehicle_state.get_D()

    #generate alternative targets:
    target_state_set = []
    for t in mconfig.time.get_samples():
        #longitudinal movement: goal is to reach velocity
        for vel in mconfig.vel.get_samples():
            s_target = [0,-vel,0] #pos not relevant
            #lateral movement
            for di in lane_config.get_samples():
                d_target = [di,0,0]
                #add target
                target_state_set.append((s_target,d_target,t))

    best, best_target, candidates, unfeasible = optimized_trajectory(
        vehicle_state, target_state_set, mconfig, lane_config, vehicles, pedestrians, static_objects,
        s_solver=quartic_polynomial_solver)
    return best, candidates, unfeasible

def plan_following(vehicle_state:VehicleState, mconfig:MFollowConfig, lane_config:LaneConfig, vehicles=None, pedestrians=None, static_objects=None):
    """
    VEHICLE FOLLOWING
    Moving target point, requiring a certain temporal safety distance to the vehicle ahead (constant time gap law).
    Predict leading vehicle (assume constant acceleration)
    """
    # log.info('Maneuver:  Vehicle Following {}'.format(mconfig.target_vid))

    s_start = vehicle_state.get_S()
    d_start = vehicle_state.get_D()
    target_vid = mconfig.target_vid
    target_t = mconfig.time.value
    time_gap = mconfig.time_gap
    acc = -mconfig.decel.value

    #Pre conditions. Can the vehicle be followed?
    if target_vid is None:
        return
    leading_vehicle = vehicles[target_vid]
    #Is target ahead?
    if (s_start[0] >= leading_vehicle.state.s):
        return
    #Is target on the same lane?
    if lane_config.get_current_lane(d_start[0]).id != lane_config.get_current_lane(leading_vehicle.state.d).id:
        log.warn("Leading vehicle {} is on a different lane.".format(target_vid))
        return

    # check if we need to deccel to increase gap, for the case when both vehicles
    # have positive velocity.
    dist_between_vehicles = leading_vehicle.state.s - VEHICLE_RADIUS - s_start[0] - VEHICLE_RADIUS
    ttc = dist_between_vehicles / abs(s_start[1]) if s_start[1] != 0 else float('inf')
    following_too_close = ttc < mconfig.time_gap

    #generate alternative targets:
    target_state_set = []

    for t in mconfig.time.get_samples():
        #longitudinal movement: goal is to keep safe distance from leading_vehicle
        s_lv = leading_vehicle.future_state(t)[:3]
        s_target = [0,0,0]

        # If leading vehicle is slower than some threshold velocity, our
        # target is to stop.
        if abs(s_lv[1]) > 1.5:
            # NOTE: this was used to explicitly decelerate the vehicle when it got too close, after
            # fixing traj generation this doesn't seem needed...
            if False:
                # decelerate for t seconds to increase time gap
                # s_target[2] = acc
                # s_target[1] = s_start[1] + acc * t
                # s_target[0] = s_lv[0] - (time_gap * s_target[1])

                # calculate decceleration needed for t seconds to achieve time gap
                acc_target = (s_lv[0] - s_start[1]*(t + time_gap) - s_start[0]) / (t*(time_gap + t))
                s_target[2] = acc_target
                s_target[1] = s_start[1] + acc_target * t
                s_target[0] = s_start[0] + s_start[1]*t + acc_target*t*t
                # log.info("Following too closely. Chose target speed of {}".format(s_target[1]))

                # calculate exactly how long to decelerate for to achieve desired time gap
                # roots = np.roots([acc, s_start[1] - s_lv[1] + time_gap*acc, s_start[1] * time_gap])
                # time_to_gap = max(roots)
                # s_target[2] = acc
                # s_target[1] = s_start[1] + acc * time_to_gap
                # s_target[0] = s_lv[0] - (time_gap * s_target[1])

                # calculate target v to make time gap 2s, keeping DISTANCE constant
                # s_target[1] = dist_between_vehicles / time_gap
                # s_target[0] = s_lv[0] - (time_gap * s_target[1])
                # s_target[2] = (s_target[1] - s_start[1]) / t
            else:
                # match leading vehicle speed
                # log.info("matching leading speed {}".format(s_lv[1]))
                s_target[0] = s_lv[0] - (time_gap * s_lv[1])
                s_target[1] = s_lv[1]
                s_target[2] = s_lv[2]
        else:
            # stop some meters behind stopped vehicle
            s_target[0] = leading_vehicle.state.s - 5 - VEHICLE_RADIUS * 2
            # log.info("leading stopped")
        #lateral movement
        for di in lane_config.get_samples():
            d_target = [di,0,0]
            #add target
            target_state_set.append((s_target,d_target,t))

    best, best_target, candidates, unfeasible = optimized_trajectory(
        vehicle_state, target_state_set, mconfig, lane_config, vehicles, pedestrians, static_objects,
        s_solver = quintic_polynomial_solver)
    # if best:
    #     log.info("starting FOLLOW: {:.3f} {:.3f} {:.3f}".format(start_state[0], start_state[1], start_state[2]))
    #     log.info("targetting end {:.3f} {:.3f} {:.3f} at t={:.3f}".format(best_target[0][0], best_target[0][1], best_target[0][2], best_target[-1]))
    # else:
    #     log.info("No FOLLOW traj")
    return best, candidates, unfeasible


def plan_laneswerve(vehicle_state:VehicleState, mconfig:MLaneSwerveConfig, lane_config:LaneConfig, vehicles=None, pedestrians=None, static_objects=None):
    """
    LANE CHANGE SWERVE
    Swerve maneuver to another lane
    No vehicles affecting the lane change, except in case of collision detection (if Collision is on).
    """
    #print ('Maneuver: Lane Change Swerve')
    s_start = vehicle_state.get_S()
    d_start = vehicle_state.get_D()
    target_lid = mconfig.target_lid

    #Find target lane
    target_lane_config = None
    if (lane_config.id == target_lid):
        target_lane_config = lane_config
        log.warn('already in target lane {}'.format(target_lid))
        return None, None
    else:
        target_lane_config = lane_config.get_neighbour(target_lid)
    if not target_lane_config:
        log.warn('target lane {} not found, is it a neighbour lane?'.format(target_lid))
        return None, None

    #generates alternative targets:
    target_state_set = []
    for t in mconfig.time.get_samples():
        #longitudinal movement: goal is to keep velocity
        vel = s_start[1]
        s_target = [0,vel,0] #pos not relevant, no acc expected at the end
        #lateral movement
        for di in target_lane_config.get_samples():
            d_target = [di,0,0] #no lateral movement expected at the end
            target_state_set.append((s_target,d_target,t))

    best, best_target, candidates, unfeasible = optimized_trajectory(
        vehicle_state, target_state_set, mconfig, target_lane_config, vehicles, pedestrians, static_objects,
        s_solver = quartic_polynomial_solver)
    return best, candidates, unfeasible


def plan_cutin(vehicle_state:VehicleState, mconfig:MCutInConfig, lane_config:LaneConfig, vehicles=None, pedestrians=None, static_objects=None):
    """
    CUT-IN LANE SWERVE
    """
    target_id = mconfig.target_vid
    delta = mconfig.delta_s + mconfig.delta_d

    if (target_id not in vehicles):
        log.warn("Target vehicle {} is not in traffic".format(target_id))
        return None

    target_lane_config = lane_config.get_current_lane(vehicles[target_id].state.d)
    if not target_lane_config:
        log.warn("Target vehicle {} is not in an adjacent lane".format(target_id))
        return None, None
    elif target_lane_config.id == lane_config.id:
        log.warn("Already in target lane")
        return None, None

    # List[(target s, target d, t)]
    target_state_set = []
    for t in mconfig.time.get_samples():
        #main goal is relative to target vehicle predicted final position
        state_relative_to = vehicles[target_id].future_state(t)
        # log.info("cuttee future state at t={}: {:.3f} {:.3f} {:.3f}".format(t, state_relative_to[0], state_relative_to[1], state_relative_to[2]))
        goal_state_relative = np.array(state_relative_to) + np.array(delta)
        goal_state_relative[0] += 2 * VEHICLE_RADIUS
        # log.info("with delta: {:.3f} {:.3f} {:.3f}".format(goal_state_relative[0], goal_state_relative[1], goal_state_relative[2]))

        # sample in s
        s_samples = []
        for i in range(3):
            ds = (mconfig.delta_s_sampling[i][0] / 100) * goal_state_relative[i]
            s_samples.append(
                np.linspace(goal_state_relative[i] - ds, goal_state_relative[i] + ds, num=mconfig.delta_s_sampling[i][1])
            )
        # log.info("s samples {}".format(s_samples))
        for s in s_samples[0]:
            for s_vel in s_samples[1]:
                for s_acc in s_samples[2]:
                    s_target = np.array([s, s_vel, s_acc])
                    for di in target_lane_config.get_samples():
                        # no lateral movement expected at the end
                        d_target = [di, 0, 0]
                        target_state_set.append((s_target, d_target, t))

    # log.info("targets: {}".format(["({:.3f} {:.3f} {:.3f})".format(t[0][0], t[0][1], t[0][2]) for t in target_state_set]))
    best, best_target, candidates, unfeasible = optimized_trajectory(
        vehicle_state, target_state_set, mconfig, target_lane_config, vehicles, pedestrians, static_objects,
        s_solver = quintic_polynomial_solver)
    return best, candidates, unfeasible


def plan_stop(vehicle_state:VehicleState, mconfig:MStopConfig, lane_config:LaneConfig, vehicles=None, pedestrians = None, static_objects=None):
    """
    STOP
    Stop can be a stop request by time and/or distance from current pos.
    Or optionally have a specific target position to stop (stop line, before an object, etc).
    """
    #log.info('PLAN STOP' + str(mconfig.pos))

    #start
    s_start = vehicle_state.get_S()
    d_start = vehicle_state.get_D()

    #Already stopped?
    if (abs(s_start[1]) <= 0.05):
        # (s_coef, d_coef, t)
        return ([np.array([vehicle_state.s, 0, 0, 0, 0, 0]), np.array([vehicle_state.d, 0, 0, 0, 0, 0]), 0]), None, None
    

    #adjust target pos to vehicle length
    target_pos = mconfig.pos - VEHICLE_RADIUS -  mconfig.distance

    #adjust target pos to possible dynamic elements:
    lv = get_leading_vehicle(vehicle_state,lane_config,vehicles)
    if lv:
        max_pos = lv.state.s - VEHICLE_RADIUS*3
        if target_pos > max_pos:
            print('Stop target {} adjusted to lead {}. New target {}'.format(target_pos, lv.state.s, max_pos))
            target_pos = max_pos
    
    #assuming uniform acceleration
    stop_distance = target_pos - s_start[0]
    expected_time = (2*stop_distance) / (vehicle_state.s_vel)
    #larget bound >40% recommended for safely finding a suitable stop time
    mconfig.time = MP(expected_time,40,6) 
    
    # within a certain distance generating new trajectory doesn't make sense
    if abs(target_pos - vehicle_state.s) < 1:
        log.warn('Stop target position is too close. diff={}'.format(target_pos - vehicle_state.s))
        #mconfig.type = MStopConfig.Type.NOW
        return None, None, None
        

    # when vehicle is past the target point, switch to STOP NOW
    if (target_pos - vehicle_state.s) < 0:
        log.warn('Stop target position behind. diff={}. Switch to Stop NOW'.format(target_pos - vehicle_state.s))
        mconfig.type = MStopConfig.Type.NOW
        
    
    #solver depends on stop type
    if mconfig.type == MStopConfig.Type.NOW:
        s_solver = quartic_polynomial_solver
    else:
        s_solver = quintic_polynomial_solver


    #targets
    target_state_set = []
    #generates alternative targets
    for t in mconfig.time.get_samples():
        #longitudinal movement: goal is to reach vel and acc 0
        s_target = [target_pos, 0, 0]
        #lateral movement
        for di in lane_config.get_samples():
            d_target = [di, 0, 0]
            #add target
            target_state_set.append((s_target, d_target, t))

    best, best_target, candidates, unfeasible = optimized_trajectory(
        vehicle_state, target_state_set, mconfig, lane_config, vehicles, pedestrians, static_objects,
        s_solver = s_solver)
    # if best:
    #     log.info("target stop {}, actual {}".format(target_pos, to_equation(best[0])(best[-1])))
    return best, candidates, unfeasible


#===TRAJECTORY OPTIMIZATION ===

def optimized_trajectory(vehicle_state:VehicleState, target_state_set, 
                        mconfig, lane_config:LaneConfig, 
                        vehicles, pedestrians, static_objects, s_solver):
    """
    Generates and select the best trajectory for the maneuver.
    Returns the resulting trajectory and a list of candidates for debug purposes.
    """
    start_state = np.concatenate([vehicle_state.get_S(),vehicle_state.get_D()])
    #find trajectories
    trajectories = []
    #quintic_maneuvers = [Maneuver.M_CUTIN, Maneuver.M_FOLLOW, Maneuver.M_STOP]
    #traj_solver = find_trajectory if mconfig.mkey in quintic_maneuvers else \
    #    lambda x: find_trajectory(x, s_solver=quartic_polynomial_solver)

    traj_solver = lambda x: find_trajectory(x, s_solver)
    trajectories = list(map(traj_solver, zip(itertools.repeat(start_state), target_state_set)))

    #evaluate feasibility
    f_trajectories_and_targets = []
    uf_trajectories_and_targets = []
    for traj, target_state in zip(trajectories, target_state_set):
        if maneuver_feasibility(start_state, traj, target_state, mconfig, lane_config, vehicles, pedestrians, static_objects):
            f_trajectories_and_targets.append((traj, target_state))
        else:
            uf_trajectories_and_targets.append((traj, target_state))
    #print ("{} feasible trajectories from {}".format( len(f_trajectories_and_targets), len(trajectories)))
    if len(f_trajectories_and_targets) == 0:
        log.error("No feasible trajectory to select")
        #log.info(str(start_state) + str(mconfig.time))
        return None, None, [], trajectories

    #select best by total cost
    # best = min(f_trajectories, key=lambda traj: maneuver_cost(start_state, traj, mconfig, lane_config, vehicles, static_objects))
    best = min(f_trajectories_and_targets, key=lambda pair:
               maneuver_cost(start_state, pair[0], pair[1], mconfig, lane_config, vehicles, pedestrians, static_objects))

    # returning best, target, candidates (feasible trajectories), and unfeasible for debug
    return best[0], best[1], [f[0] for f in f_trajectories_and_targets], [f[0] for f in uf_trajectories_and_targets]

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
