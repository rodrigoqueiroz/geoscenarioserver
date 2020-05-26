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

from LaneletTest import *

if __name__ == "__main__":
    #SIM CONFIG
    TIMEOUT = 30               #timeout in [s]
    FRAME_RATE = 30            #Global tick rate
    show_dashboard = False      #plot vehicles and trajectories. Optional when running with Ureal engine.
    centerplot_veh_id = 1
    #
    sync_global   = TickSync(rate=FRAME_RATE, realtime = True, block=True, verbose=True, label="EX")
    sync_global.set_timeout(TIMEOUT)

    # test class for parsing and holding map
    test_map = LaneletTest()

    # PROBLEM SETUP
    # Problem setup can be defined directly, or using GeoScenario XML files (GSParser)
    traffic = SimTraffic()
    traffic.set_map(test_map)
   #traffic.add_remote_vehicle( 99, 'Ego', [0.0,0.0,0.0, 1.0,0.0,0.0])
    traffic.add_vehicle( 1, 'V1', [15.0,0.0,0.0, 7.0,0.0,0.0], BT_VELKEEPING)
    #traffic.add_vehicle( 2, 'V2', [20.0,0.0,0.0, 2.0,0.0,0.0], BT_VELKEEPING)
    #traffic.add_vehicle( 3, 'V3', [-10.0,0.0,0.0, 2.0,0.0,0.0], BT_VELKEEPING)


    exit(1)
    
    #GUI / Debug screen
    dashboard = DashBoard()
    if (show_dashboard):
        dashboard.create()
    
    #SIM EXECUTION START
    print ('SIMULATION START')
    traffic.start()
    while sync_global.tick():
        try:
            #Update Traffic
            traffic.tick(
                sync_global.tick_count, 
                sync_global.delta_time,
                sync_global.sim_time
            )
            #Update Dashboard (if visible)
            dashboard.update(traffic, centerplot_veh_id)
        except KeyboardInterrupt:
            break
        
    traffic.stop_all()    
    dashboard.quit()
    #SIM END
    print('SIMULATION END')
