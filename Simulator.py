#!/usr/bin/env python
#rqueiroz@gsd.uwaterloo.ca
#d43sharm@edu.uwaterloo.ca
# ---------------------------------------------
# SIMULATOR
# Problem Setup and Simulation Loop
# --------------------------------------------

from TickSync import TickSync
from SimTraffic import *
from sv.DashBoard import *
from util.Constants import *

if __name__ == "__main__":
    sync_global   = TickSync(rate=FRAME_RATE, realtime = True, block=True, verbose=False, label="EX")
    sync_global.set_timeout(TIMEOUT)
    # PROBLEM SETUP
    # Problem setup can be defined directly, or using GeoScenario XML files (GSParser)
    traffic = SimTraffic()
    traffic.set_map('laneletmap.osm')
    #traffic.add_remote_vehicle( 99, 'Ego', [0.0,0.0,0.0, 1.0,0.0,0.0])
    traffic.add_vehicle( 1, 'V1', [00.0,0.0,0.0, 2.0,0.0,0.0], BT_VELKEEP)
    traffic.add_vehicle( 2, 'V2', [20.0,0.0,0.0, 0.0,0.0,0.0], BT_VELKEEP)
    #traffic.add_vehicle( 3, 'V3', [0.0,0.0,0.0, 2.0,0.0,0.0], BT_VELKEEP)
    
    
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
