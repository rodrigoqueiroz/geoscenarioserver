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
            #Update Dashboard (if visible)
            dashboard.update(traffic, centerplot_veh_id)
            #Write Shared Memory
            if(publish_pose_shm):
                pass

        except KeyboardInterrupt:
            break
        
    #END
    dashboard.quit()
    if(publish_pose_shm):
        pass

    print('SIMULATION END')
