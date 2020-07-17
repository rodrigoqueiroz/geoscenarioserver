#!/usr/bin/env python
#rqueiroz@uwaterloo.ca
#d43sharm@uwaterloo.ca


from sv.ManeuverConfig import *
from SimConfig import *
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

def cutin_completed(vehicle_state, lane_config:LaneConfig, mconfig:MCutInConfig, traffic_vehicles):
    target_lane_config = lane_config.get_current_lane(traffic_vehicles[mconfig.target_vid].vehicle_state.d)
    if not target_lane_config:
        print("Target vehicle {} is not in an adjacent lane".format(mconfig.target_vid))
        return None, None
    
    return lane_swerve_completed(vehicle_state, lane_config, MLaneSwerveConfig(target_lane_config.id))

def can_perform_lane_change():
    return True

# no way of knowing if passed goal
# def has_reached_goal(vehicle_state, goal_point, threshold=2):
#     to_goal = np.array(goal_point) - np.array([vehicle_state.x, vehicle_state.y])
#     sqr_distance = np.dot(to_goal, to_goal)
#     return sqr_distance < threshold*threshold

def has_reached_goal_frenet(vehicle_state, goal_point, threshold=2):
    return False if not goal_point else goal_point[0] - vehicle_state.s < threshold

def get_vehicle_ahead(vehicle_state, lane_config, vehicles, threshold=4):
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

def is_stopped(traffic_vehicle):
    return traffic_vehicle.vehicle_state.s_vel == 0

def is_slow_vehicle(subject_vehicle, traffic_vehicle):
    return subject_vehicle.s_vel > traffic_vehicle.vehicle_state.s_vel

def reached_acceptance_gap(vehicle_state, lane_config, vehicles, threshold=1):
    ''' Analyzes (frenet coordinates) whether is there an adversary vehicle
    (adversary_vehicle) ahead of subject vehicle (subject_vehicle) 
    sharing the same lane, within a threshold (frenet s-plane). In case
    of multiple vehicles, it returns the closest one.
    @return (vid, sv.SV.Vehicle)'''

    subject_vehicle_state = vehicle_state
    s_current_lane = lane_config.get_current_lane(subject_vehicle_state.d)
    reached = True
    for vid, adversary_vehicle in vehicles.items():
        adversary_vehicle_state = adversary_vehicle.vehicle_state

        a_current_lane = lane_config.get_current_lane(adversary_vehicle_state.d)
        if (a_current_lane is None or a_current_lane.id - 1 != s_current_lane): continue #makes sure not to compare with irrelevant vehicles (-1 is hardcoded)

        if subject_vehicle_state.s - VEHICLE_RADIUS < adversary_vehicle_state.s + VEHICLE_RADIUS + threshold: 
            reached = False

    return reached

def is_in_following_range(self_id, vehicle_state, other_vehicles, lane_config:LaneConfig, time_gap=5):
    is_following = False
    leading_vid = None

    closest_dist = float('inf')
    cur_lane = lane_config.get_current_lane(vehicle_state.d)

    for vid, traffic_vehicle in other_vehicles.items():
        other_vehicle_lane = lane_config.get_current_lane(traffic_vehicle.vehicle_state.d)
        # TODO: need a function to get all vehicles in current lane
        if other_vehicle_lane and other_vehicle_lane.id == cur_lane.id:
            dist = traffic_vehicle.vehicle_state.s - VEHICLE_RADIUS - vehicle_state.s - VEHICLE_RADIUS
            ttc = dist / abs(vehicle_state.s_vel) if vehicle_state.s_vel != 0 else float('inf')
            
            # if self_id == 2:
            #     is_following = True
            #     leading_vid = vid
            # if not moving, determine if too close (one car width's apart)
            if (0 < dist < VEHICLE_RADIUS * 10) or (0 <= ttc < time_gap):
                if dist < closest_dist:
                    # TODO follow not working? time gap doesn't seem to be correct
                    # print("{} is leading by {}".format(vid, ttc))
                    is_following = True
                    leading_vid = vid
    
    return is_following, leading_vid

#def ttc(self_id, vehicle_state, other_vehicles, lane_config:LaneConfig):
#def range(self_id, vehicle_state, other_vehicles, lane_config:LaneConfig):

def has_passed_enough_time(ref_time, curr_time, threshold):
    return ref_time - curr_time > threshold