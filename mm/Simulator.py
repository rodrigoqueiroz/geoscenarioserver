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
from Constants import *

from LaneletTest import *

if __name__ == "__main__":
    sync_global   = TickSync(rate=FRAME_RATE, realtime = True, block=True, verbose=False, label="EX")
    sync_global.set_timeout(TIMEOUT)
    # PROBLEM SETUP
    # Problem setup can be defined directly, or using GeoScenario XML files (GSParser)
    traffic = SimTraffic()
    traffic.set_map('laneletmap.osm')
   #traffic.add_remote_vehicle( 99, 'Ego', [0.0,0.0,0.0, 1.0,0.0,0.0])
    traffic.add_vehicle( 1, 'V1', [15.0,0.0,0.0, 7.0,0.0,0.0], BT_VELKEEPING)
    #traffic.add_vehicle( 2, 'V2', [20.0,0.0,0.0, 2.0,0.0,0.0], BT_VELKEEPING)
    #traffic.add_vehicle( 3, 'V3', [-10.0,0.0,0.0, 2.0,0.0,0.0], BT_VELKEEPING)

    lt = LaneletTest()

    exit(1)
    
    #GUI / Debug screen
    dashboard = DashBoard()
    if (SHOW_DASHBOARD):
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
            dashboard.update(traffic, PLOT_VID)
        except KeyboardInterrupt:
            break
        
    traffic.stop_all()    
    dashboard.quit()
    #SIM END
    print('SIMULATION END')
