#!/usr/bin/env python
#rqueiroz@gsd.uwaterloo.ca
# ---------------------------------------------
# SIMULATOR
# Problem Setup and Simulation Loop
# --------------------------------------------

import numpy as np
import random
from CostFunctions import *
from Constants import *
from Plots import *
from SV import *


SHOW_PLOT = False

# PROBLEM SETUP

def main():

    #Problem Setup

    #free lane change
    #start_state = [0,   30, 0, 3, 0, 0]      #[s,sd,sdd,d,dd,ddd] at t=0
    #goal_state =  [100, 30, 0, 1, 0, 0]      #[s,sd,sdd,d,dd,ddd] at t=T
    #T = 8.0                                  #duration of lane change maneuver
    #trajectory = ST_LaneChange(start_state, goal_state, T) #returns tuple (s[], d[], t)
    
    #cut-in lane change
    #start_state = [30, 30, 0, 3, 0, 0]      #[s,sd,sdd,d,dd,ddd] at t=0
    #goal_state =  [100, 20, 0, 1, 0, 0]     #[s,sd,sdd,d,dd,ddd] at t=T
    #T = 8.0                                 # duration of cut in maneuver
    #vehicles = {0:Vehicle([0,20,0, 1,0,0])} #dictionary with at least one vehicle (target) with initial state [s,sd,sdd,d,dd,ddd]
    #target_id = 0                           #id of target vehicle (ego)
    #delta = [10, 0, 0, 0, 0 ,0]             #target offset between vehicle and target vehicle (ego) [s,sd,sdd,d,dd,ddd]
    #trajectory = ST_CutIn(start_state, goal_state, T, vehicles, delta, target_id) #returns tuple (s[], d[], t)
    
    #cut-in lane change optimized
    """  
    start_state = [30, 30, 0, 3, 0, 0]      #[s,sd,sdd,d,dd,ddd] at t=0
    T = 8.0                                 #duration cut in maneuver
    vehicles = {0:Vehicle([0,20,0, 1,0,0])} #dictionary with at least one vehicle (target) with initial state [s,sd,sdd,d,dd,ddd]
    target_id = 0                           #id of target vehicle (ego)
    delta = [10, 0, 0, 0, 0 ,0]             #target offset between vehicle and target vehicle (ego) [s,sd,sdd,d,dd,ddd]
    delta_cross = [10, 0, 0, 0, 0 ,0]       #target offset between vehicle and target vehicle at crossing lane point (projected gap)
    trajectories, best = OT_CutIn(start_state, delta, T, vehicles,target_id,True,True) #returns (s[], d[], t)
    #plot
    plot_multi_trajectory(trajectories,best,vehicles,False,True)
    plot_sd(best) """

    #Sim HV 
    #start_state = [0, 14, 0, 6, 0, 0]      #[s,sd,sdd,d,dd,ddd] at t=0
    #hv = SV(start_state)
    #flow params
    #target_vel = 10

    #cut in params
    #target_id = 0
    #delta = [5, 10, 0, 0, 0 ,0]
    #T = 2.5                                  #duration of lane change maneuver

    #follow params
    #target_id = 0
    #T = 2                                  #duration of lane change maneuver

    #traffic (including ego)
    #ego = Vehicle([50,10,0, 5,0,0])
    #v1 = Vehicle([10,10,0, 2,0,0])
    #v2 = Vehicle([40,12,0, 1,0,0])
    #vehicles = {0:ego,1:v1,2:v2}
    #vehicles = {0:ego}

    #--------NEWs
    #Sim HV
    sim_vehicles = {}
    #TODO: change starting state to location in sim coordinate
    vid = 0
    v1 = SV(id = vid, start_state = [0.0,14.0,0.0, 10.0,0.0,0.0]) #14 +- 50km/h
    #v1.setbehavior(btree=BT_FOLLOW, target_id=1)
    # v1.setbehavior(btree=BT_STOP) 
    v1.setbehavior(btree=BT_VELKEEPING)
    sim_vehicles[vid] = v1
    
    # TODO: ask how d works, and if every vehicle will have its own transform out of frenet
    vid = 1
    v2 = SV(id = vid, start_state = [0.0,14.0,0.0, 0.0,0.0,0.0]) #14 +- 50km/h
    v2.setbehavior(btree=BT_VELKEEPING)
    sim_vehicles[vid] = v2

    #v2 = SV(id = 1, start_state = [100,10,0, 2,0,0]) 
    #v2.setbehavior(btree=BT_VELKEEPING) 
    #sim_vehicles[1] = v2
    #v3 = SV(id,[0,10,0, 2,0,0]) 
    #sim_vehicles[id] = v3

    #Test params
    MAX_SIM_TIME = 100.0
    area = 100.0  #animation length in m
    sim_time = 0
    time_step = 0.1
    centerplot_veh = 0
    
    #Sim Loop
    while sim_time<=MAX_SIM_TIME:
        try:
            #pass time
            sim_time += time_step

            if SHOW_PLOT:
                #plot
                plt.cla()
                plt.grid(True)
                plot_road()
                plt.xlim(sim_vehicles[centerplot_veh].s_pos - area, sim_vehicles[centerplot_veh].s_pos + area)

            #update agents
            for svid in sim_vehicles:
                sim_vehicles[svid].tick(time_step, sim_vehicles)
                sim_vehicles[svid].plot()
            
            if SHOW_PLOT:
                plt.pause(0.0001)
                #plot_hv(hv,best, None, 'blue')
                #plot_vehicles(vehicles,T, 'green')
                #plt.xlim(hv.start_state[0] - area, hv.start_state[0] + area)
                #plt.ylim(hv.start_state[3] - area, hv.start_state[3] + area)
        except KeyboardInterrupt:
            break

    #plt.show() #blocks UI



if __name__ == "__main__":
	main()
