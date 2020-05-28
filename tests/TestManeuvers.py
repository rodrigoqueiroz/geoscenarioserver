#from __future__ import annotations  #Must be first Include. Will be standard in Python4
#from dataclasses import dataclass
#import numpy as np
#import random
from sv.ManeuverConfig import *
from sv.ManeuverModels import *
from sv.VehicleState import *

def AllTests():
    print("Test Velocity Keeping")
    #mconfig = MVelKeepConfig(MP(40,10,2),MP(10))
    mconfig = MVelKeepConfig()
    print(mconfig.vel.get_linear_samples())
    print(mconfig.vel.get_uniform_samples())
    print(mconfig.vel.get_normal_samples())
    #print(vk.time.nsamples)
    vehicle_frenet_state = [0.0,0.0,0.0, 0.0,0.0,0.0]
    lane_config = LaneConfig(1,30,4,0)
    
    traj, cand = plan_maneuver( M_VELKEEP, mconfig, vehicle_frenet_state, lane_config, None)
    print("Trajectory")
    print(traj)


    print("Test Lane Swerve")
    mconfig = MLaneSwerveConfig(target_lid=2)
    vehicle_frenet_state = [0.0,0.0,0.0, 0.0,0.0,0.0]
    lane_config = LaneConfig(1,30,4,0)
    upper_lane_config = LaneConfig(2,30,8,4)
    lane_config.set_left_lane(upper_lane_config)
    
    traj, cand = plan_maneuver( M_LANESWERVE, mconfig, vehicle_frenet_state, lane_config, None)
    print("Trajectory")
    print(traj)
    
