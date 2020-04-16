#!/usr/bin/env python
#rqueiroz@gsd.uwaterloo.ca
#d43sharm@edu.uwaterloo.ca
# ---------------------------------------------
# SIMULATOR
# Problem Setup and Simulation Loop
# --------------------------------------------

import numpy as np
import random
from CostFunctions import *
from Constants import *
from DashBoard import *
from SV import *
from TickSync import TickSync

if __name__ == "__main__":

    #SIM CONFIG
    show_dashboard = False       #plot vehicles and trajectories. Optional when running with Ureal engine.
    publish_pose_shm = True     #write vehicle Pose in shared memory for Unreal Engine
    centerplot_veh_id = 0
    #

    # PROBLEM SETUP
    #Sim HV
    sim_vehicles = {}
    #TODO: change starting state to location in sim coordinate
    v1 = SV(id = 0, start_state = [0.0,0.0,0.0, 2.0,0.0,0.0]) #14 +- 50km/h
    #v1.setbehavior(btree=BT_FOLLOW, target_id=1)
    #v1.setbehavior(btree=BT_STOP) 
    v1.setbehavior(btree=BT_VELKEEPING) 
    sim_vehicles[0] = v1

    #v2 = SV(id = 1, start_state = [100,10,0, 2,0,0]) 
    #v2.setbehavior(btree=BT_VELKEEPING) 
    #sim_vehicles[1] = v2
    
    #v3 = SV(id,[0,10,0, 2,0,0]) 
    #sim_vehicles[id] = v3
    
    #SIM EXECUTION
    dashboard = DashBoard()
    if (show_dashboard):
        dashboard.create()

    sync_global   = TickSync(rate=30, block=True, verbose=True, label="EX") #Global Tick blocks if too fasst
    sync_global.set_timeout(60)  #timeout in [s]s
    
    print ('SIMULATION START')
    while sync_global.tick():
        try:
            #TODO: Update Ego Pose

            #Update Dynamic Agents
            for svid in sim_vehicles:
                sim_vehicles[svid].tick(sync_global.tick_delta_time, sim_vehicles)
            
            #Update Dashboard (if visible)
            dashboard.update(sim_vehicles,centerplot_veh_id)
        except KeyboardInterrupt:
            break
        
    #END
    dashboard.quit()
    if(publish_pose_shm):
        pass

    print('SIMULATION END')
    #plt.show() #This function blocks UI. Use with caution