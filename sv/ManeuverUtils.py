#!/usr/bin/env python

from sv.ManeuverConfig import *
from util.Constants import *

# this messes up frenet state because itll report we have not lane changed
# even tho new reference path is calculated
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
    pass
