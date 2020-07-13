#!/usr/bin/env python

from sv.ManeuverConfig import *
from util.Constants import *
import numpy as np

def lane_swerve_completed(vehicle_state, lane_config:LaneConfig, mconfig:MLaneSwerveConfig):
    current_lane = None
    if mconfig.target_lid == 1: # left
        # check if right border of vehicle has crossed right border of lane
        current_lane = lane_config.get_current_lane(vehicle_state.d - VEHICLE_RADIUS)
    elif mconfig.target_lid == -1: # right
        current_lane = lane_config.get_current_lane(vehicle_state.d + VEHICLE_RADIUS)
    else: # target_lid is None or 0
        print("WARNING: Lane swerve target_lid {}".format(mconfig.target_lid))
        current_lane = lane_config
    
    return current_lane.id == mconfig.target_lid

def can_perform_lane_change():
    return True

# no way of knowing if passed goal
# def has_reached_goal(vehicle_state, goal_point, threshold=2):
#     to_goal = np.array(goal_point) - np.array([vehicle_state.x, vehicle_state.y])
#     sqr_distance = np.dot(to_goal, to_goal)
#     return sqr_distance < threshold*threshold

def has_reached_goal_frenet(vehicle_state, goal_point, threshold=2):
    return False if not goal_point else goal_point[0] - vehicle_state.s < threshold

def get_vehicle_ahead(vehicle_state, lane_config, vehicles, threshold=10):
    ''' Analyzes (frenet coordinates) whether is there an adversary vehicle
    (adversary_vehicle) ahead of subject vehicle (subject_vehicle) 
    sharing the same lane, within a threshold (frenet s-plane). In case
    of multiple vehicles, it returns the closest one.
    @return (vid, sv.SV.Vehicle)'''

    subject_vehicle_state = vehicle_state
    s_current_lane = lane_config.get_current_lane(subject_vehicle_state.d)
    dist = float('inf')
    nearest = list()
    for vid, adversary_vehicle in vehicles.items():
        a_current_lane = lane_config.get_current_lane(adversary_vehicle.vehicle_state.d)
        
        #if they are both in the same lane
        if s_current_lane == a_current_lane: 
            # and the adv is ahead of the subj (within the thresh)
            if subject_vehicle_state.s < adversary_vehicle.vehicle_state.s and adversary_vehicle.vehicle_state.s < subject_vehicle_state.s + threshold: 
                diff = adversary_vehicle.vehicle_state.s - subject_vehicle_state.s
                if diff < dist:
                    dist = diff
                    nearest = [vid,adversary_vehicle]

    return nearest

def is_stopped(vehicle):
    return vehicle.vehicle_state.s_vel == 0
