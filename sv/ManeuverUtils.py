#!/usr/bin/env python3
#rqueiroz@uwaterloo.ca
#d43sharm@uwaterloo.ca
# ---------------------------------------------
# Util functions for decision-making in the frenet frame.
# --------------------------------------------


from sv.ManeuverConfig import *
from SimConfig import *
import numpy as np
import glog as log

def lane_swerve_completed(vehicle_state, lane_config:LaneConfig, mconfig:MLaneSwerveConfig):
    current_lane = None
    if mconfig.target_lid == 1: # left
        # check if right border of vehicle has crossed right border of lane
        current_lane = lane_config.get_current_lane(vehicle_state.d - VEHICLE_RADIUS)
    elif mconfig.target_lid == -1: # right
        current_lane = lane_config.get_current_lane(vehicle_state.d + VEHICLE_RADIUS)
    else: # target_lid is None or 0
        log.warn("WARNING: Lane swerve completed into target_lid {}".format(mconfig.target_lid))
        current_lane = lane_config

    return current_lane.id == mconfig.target_lid

def cutin_completed(vehicle_state, lane_config:LaneConfig, mconfig:MCutInConfig, traffic_vehicles):
    target_lane_config = lane_config.get_current_lane(traffic_vehicles[mconfig.target_vid].vehicle_state.d)
    if not target_lane_config:
        log.warn("Target vehicle {} is not in an adjacent lane".format(mconfig.target_vid))
        return None, None

    return lane_swerve_completed(vehicle_state, lane_config, MLaneSwerveConfig(target_lane_config.id))

def lane_swerve_or_cutin_completed(vehicle_state, lane_config:LaneConfig, mconfig, traffic_vehicles):
    if type(mconfig) == MLaneSwerveConfig:
        return lane_swerve_completed(vehicle_state, lane_config, mconfig)
    elif type(mconfig) == MCutInConfig:
        return cutin_completed(vehicle_state, lane_config, mconfig, traffic_vehicles)
    return False

def can_perform_lane_change():
    return True

# no way of knowing if passed goal
# def has_reached_goal(vehicle_state, goal_point, threshold=2):
#     to_goal = np.array(goal_point) - np.array([vehicle_state.x, vehicle_state.y])
#     sqr_distance = np.dot(to_goal, to_goal)
#     return sqr_distance < threshold*threshold

#def has_reached_goal_frenet(vehicle_state, goal_point, threshold=2):
#    return False if not goal_point else goal_point[0] - vehicle_state.s < threshold

def has_reached_goal_frenet(vehicle_state, frenet_goal_point, threshold=20, reverse=False):
    """ Checks if the vehicle has reached or passed the goal point in the frenet frame.
        @param frenet_goal_point:  Arraylike (s,d) goal position in the vehicle's frenet frame
    """
    goal_s = 0 if reverse else frenet_goal_point[0]
    direction = -1 if reverse else 1
    # A distance to goal with the same sign as direction means we've reached and passed it
    return direction * (goal_s - vehicle_state.s) < threshold

def is_in_following_range(self_id, vehicle_state, other_vehicles, lane_config:LaneConfig, time_gap=5, distance_gap=30):
    """ Determines whether there is a vehicle in front of a given vehicle in the same lane and within a specified
        time or distance gap.
        @param time_gap:        The time gap below which a vehicle is considered to be following a leading vehicle.
        @param distance_gap:    The distance at which a vehicle is considered to be following a leading vehicle,
                                regardless of the time gap to the leading vehicle. This should be around the stopping
                                distance of SVs.
    """
    log.check_notnone(lane_config)

    is_following = False
    leading_vid = None

    leading_vehicle = get_leading_vehicle(vehicle_state, lane_config, other_vehicles)

    if leading_vehicle is not None:
        dist = leading_vehicle.vehicle_state.s - VEHICLE_RADIUS - vehicle_state.s - VEHICLE_RADIUS
        if dist < 0:
            log.error("distance to leading vehicle zero!")

        time_to_leader = dist / vehicle_state.s_vel if vehicle_state.s_vel != 0 else float('inf')

        # Enter following range if below threshold for distance gap or time gap.
        if (dist < distance_gap) or (0 <= time_to_leader < time_gap):
            # print("{} is leading by {}".format(leading_vehicle.vid, current_gap))
            is_following = True
            leading_vid = leading_vehicle.vid

    return is_following, leading_vid

def is_lane_occupied(vehicle_state, lane_config, traffic_vehicles, threshold=50):
    log.check_notnone(lane_config)

    smin = vehicle_state.s - threshold
    smax = vehicle_state.s + threshold
    vehicles_in_lane = list(filter(
        lambda v: smin < v.vehicle_state.s < smax,
        get_vehicles_in_lane(lane_config, traffic_vehicles)
    ))

    return len(vehicles_in_lane) != 0

def get_vehicles_in_lane(lane_config, traffic_vehicles):
    log.check_notnone(lane_config)

    vehicles = []
    for vid, traffic_vehicle in traffic_vehicles.items():
        other_vehicle_lane = lane_config.get_current_lane(traffic_vehicle.vehicle_state.d)
        if other_vehicle_lane and other_vehicle_lane.id == lane_config.id:
            vehicles.append(traffic_vehicle)
    return vehicles

# same as below, but i didn't wanna touch Ricardo's code. TODO clean this up
def get_leading_vehicle(vehicle_state, lane_config, traffic_vehicles):
    """ Gets closest vehicle in the same lane as vehicle_state.
    """
    log.check_notnone(lane_config)

    cur_lane = lane_config.get_current_lane(vehicle_state.d)
    log.check_notnone(cur_lane)

    vehicles_ahead = list(filter(
        lambda v: v.vehicle_state.s > vehicle_state.s,
        get_vehicles_in_lane(cur_lane, traffic_vehicles)
    ))

    if len(vehicles_ahead) == 0:
        return None

    return min(vehicles_ahead, key=lambda v: v.vehicle_state.s)

def get_closest_vehicle_in_lane(vehicle_state, lane_config, traffic_vehicles):
    vehicles_in_lane = get_vehicles_in_lane(lane_config, traffic_vehicles)
    if len(vehicles_in_lane) == 0:
        return None
    return min(vehicles_in_lane, key=lambda v: abs(v.vehicle_state.s - vehicle_state.s))

def reached_gap(vehicle_state, target_lane_config, traffic_vehicles, meters):
    """ determines whether `vehicle_state` is `meters` ahead of the
        nearest vehicle in the target lane.
    """
    target_vehicle = get_closest_vehicle_in_lane(vehicle_state, target_lane_config, traffic_vehicles)
    if target_vehicle is None:
        log.warn("No target vehicle in {} lane.".format('LEFT' if target_lane_config.id == 1 else 'RIGHT'))
        return True
    gap = vehicle_state.s - target_vehicle.vehicle_state.s
    return gap > meters

#def ttc(self_id, vehicle_state, other_vehicles, lane_config:LaneConfig):
#def range(self_id, vehicle_state, other_vehicles, lane_config:LaneConfig):

#Ricardo's implementation:

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
    return abs(traffic_vehicle.vehicle_state.s_vel) < 0.05

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

def has_passed_enough_time(ref_time, curr_time, threshold):
    return ref_time - curr_time > threshold

# TODO
def is_gap_reachable(vehicle, traffic_vehicles, gap_size):
    return False

# TODO
def was_the_gap_reached(vehicle, traffic_vehicles, gap_size):
    return False
