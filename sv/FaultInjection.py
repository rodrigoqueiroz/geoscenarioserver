#!/usr/bin/env python
#dinizr@chalmers.se
from random import *

velkeep_vel_error = 0
velkeep_time_error = 0
velkeep_time_lowvel_error = 0
velkeep_vel_threshold_error = 0

reverse_vel_error = 0
reverse_time_error = 0

stop_time_error = 0
stop_distance_error = 0
stop_decel_error = 0
stop_min_decel_error = 0
stop_max_decel_error = 0

stopat_time_error = 0
stopat_error = 0

follow_time_error = 0
follow_time_gap_error = 0
follow_decel_error = 0


def random_error(value, percent):
    inv = -1 if random.Random()<0.5 else 1
    ans = value * (1 - inv*percent*value)
    return ans if ans >= 0 else 0

def injectFault(mconfig):

    if (mconfig.key == Maneuver.M_VELKEEP):
        mconfig.vel.value = random_error(mconfig.vel.value, velkeep_vel_error) # vel= vel+-delta
        mconfig.time.value = random_error(mconfig.time.value, velkeep_time_error)
        mconfig.time_lowvel.value = random_error(mconfig, velkeep_time_lowvel_error)
        mconfig.vel_threshold = random_error(mconfig.vel_threshold, velkeep_vel_threshold_error)

    elif (mconfig.key == Maneuver.M_REVERSE):

        mconfig.vel.value = random_error(mconfig.vel.value, reverse_vel_error)
        mconfig.time.value = random_error(mconfig.time.value, reverse_time_error)

    elif (mconfig.key == Maneuver.M_STOP):

        mconfig.time.value = random_error(mconfig.time.value, stop_time_error)
        mconfig.distance.value = random_error(mconfig.distance.value, stop_distance_error)
        mconfig.min_decel = random_error(mconfig.min_decel, stop_min_decel_error)
        mconfig.max_decel = random_error(mconfig.max_decel, stop_max_decel_error)

    elif (mconfig.key == Maneuver.M_STOP_AT):

        mconfig.time.value = random_error(mconfig.time.value, stopat_time_error)
        mconfig.stop_pos = random_error(mconfig.time.value, stopat_error)

    elif (mconfig.key == Maneuver.M_FOLLOW):

        # mconfig.target_vid = xxx
        mconfig.time.value = random_error(mconfig.time.value, follow_time_error)
        mconfig.time_gap = random_error(mconfig.time_gap, follow_time_gap_error)
        mconfig.decel.value = random_error(mconfig.decel.value, follow_decel_error)

    elif (mconfig.key == Maneuver.M_LANESWERVE):

        # mconfig.target_lid = xxx
        mconfig.time.value = random_error(mconfig.time.value, laneswerve_time_error)

    elif (mconfig.key == Maneuver.M_CUTIN):
        # mconfig.target_vid = xxx
        # mconfig.target_lid = xxx
        mconfig.time.value = random_error(mconfig.time.value, cutin_time_error)
        mconfig.delta_s = (random_error(mconfig.delta_s[0], cutin_delta_s_s_error))
        mconfig.delta_s = (random_error(mconfig.delta_s[1], cutin_delta_s_vel_error))
        mconfig.delta_s = (random_error(mconfig.delta_s[2], cutin_delta_s_acc_error))
        mconfig.delta_d = (random_error(mconfig.delta_d[0], cutin_delta_d_s_error))
        mconfig.delta_d = (random_error(mconfig.delta_d[1], cutin_delta_d_vel_error))
        mconfig.delta_d = (random_error(mconfig.delta_d[2], cutin_delta_d_acc_error))
        

    else:
        print("[Warning]: Maneuver not found, no fault was injected.")

    return mconfig