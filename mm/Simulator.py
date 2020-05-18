#!/usr/bin/env python
#rqueiroz@gsd.uwaterloo.ca
#d43sharm@edu.uwaterloo.ca
# ---------------------------------------------
# SIMULATOR
# Problem Setup and Simulation Loop
# --------------------------------------------

from TickSync import TickSync
from DashBoard import *
from SimTraffic import *
from shared_mem.SVSharedMemory import *
from shared_mem.EgoSharedMemory import *
import threading

if __name__ == "__main__":
    #SIM CONFIG
    TIMEOUT = 20               #timeout in [s]
    FRAME_RATE = 30            #Global tick rate
    show_dashboard = True      #plot vehicles and trajectories. Optional when running with Ureal engine.
    publish_pose_shm = True    #write vehicle Pose in shared memory for Unreal Engine
    centerplot_veh_id = 1
    #
    sync_global   = TickSync(rate=FRAME_RATE, block=True, verbose=False, label="EX")
    sync_global.set_timeout(TIMEOUT)

    # SHARED MEMORY SETUP
    shared_memory = SVSharedMemory()
    ego_shared_memory = EgoSharedMemory()

    # PROBLEM SETUP
    #TODO: Load problem setup GeoScenario XML file (using GSParser)
    traffic = SimTraffic()
    traffic.set_map("laneletmap.osm")
    traffic.add_vehicle(1, [0.0,0.0,0.0, 1.0,0.0,0.0], BT_VELKEEPING)
    traffic.add_vehicle(2, [15.0,0.0,0.0, 7.0,0.0,0.0], BT_VELKEEPING)
    traffic.add_vehicle(3, [20.0,0.0,0.0, 2.0,0.0,0.0], BT_VELKEEPING)
    #traffic.add_vehicle(4, [-10.0,0.0,0.0, 2.0,0.0,0.0], BT_VELKEEPING)
    
    #GUI
    dashboard = DashBoard()
    if (show_dashboard):
        dashboard.create()
    
    #SIM EXECUTION
    print ('SIMULATION START')
    traffic.start()
    while sync_global.tick():
        try:
            #Update Traffic
            traffic.tick(
                sync_global.tick_count, 
                sync_global.tick_delta_time,
                sync_global.sim_time
            )
            #TODO: Update Ego Pose
            #if ego_shared_memory.is_connected:
            #    ego_shared_memory.read_memory()

            #Update Dynamic Agents
            #for svid in sim_vehicles:
            #    sim_vehicles[svid].tick(sync_global.tick_count, sync_global.tick_delta_time)
            
            # Write out simulator state
            #shared_memory.write_vehicle_stats(sync_global.tick_count, sync_global.tick_delta_time, sim_vehicles)

            #Update Dashboard (if visible)
            dashboard.update(traffic, centerplot_veh_id)
            #Write Shared Memory
            if(publish_pose_shm):
                pass
        
        # TODO is this necessary?
        except KeyboardInterrupt:
            break
        
    #END
    dashboard.quit()
    if(publish_pose_shm):
        pass

    print('SIMULATION END')
