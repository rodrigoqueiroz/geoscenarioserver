#!/usr/bin/env python
#rqueiroz@gsd.uwaterloo.ca
#d43sharm@uwaterloo.ca
# ---------------------------------------------
# SIMULATOR
# Problem Setup and Simulation Loop
# --------------------------------------------

from TickSync import TickSync
from SimTraffic import *
from dash.Dashboard import *
from util.Constants import *
from Mapping.LaneletMap import *
from SimConfig import SimConfig


if __name__ == "__main__":
    sync_global   = TickSync(rate=TRAFFIC_RATE, realtime = True, block=True, verbose=False, label="EX")
    sync_global.set_timeout(TIMEOUT)

    # class for parsing and holding map
    test_map = LaneletMap("scenarios/mapping_example.osm") # "scenarios/ll2_round.osm"
    # class for holding scenario settings - load from file?
    sim_config = SimConfig({
        1 : test_map.get_route(329661501650965856, 99991), # [99998, 99997, 99996, 99995, 99994, 99993, 99992, 99991],
        2 : test_map.get_route(329661501650965856, 99991), # [99998, 99997, 99996, 99995, 99994, 99993, 99992, 99991],
        })
    # for testing - grab a path to put vehicles on
    ref_path = test_map.get_global_path_for_route(sim_config.lanelet_routes[1])

    # PROBLEM SETUP
    # Problem setup can be defined directly, or using GeoScenario XML files (GSParser)
    traffic = SimTraffic()
    # set these BEFORE adding vehicles - also why not using constructor?
    traffic.set_map(test_map)
    traffic.set_sim_config(sim_config)
    #traffic.add_remote_vehicle( 99, 'Ego', [0.0,0.0,0.0, 1.0,0.0,0.0])
    #traffic.add_vehicle( 1, 'V1', [ref_path[1].x,0.0,0.0, ref_path[1].y,0.0,0.0],
    #    sim_config.lanelet_routes[1], BT_VELKEEP)
    # adding vehicle at the start of a lanelet
    traffic.add_vehicle(1, 'V1', [4.0,0.0,0.0, 0.0,0.0,0.0],
        sim_config.lanelet_routes[1], BT_VELKEEP, start_state_in_frenet=True)
    # test location
    # traffic.add_vehicle( 1, 'V1', [0,0.0,0.0, 0,0.0,0.0], BT_VELKEEP)
    # traffic.add_vehicle(2, 'V2', [8,0.0,0.0, 0.0,0.0,0.0],
    #     sim_config.lanelet_routes[2], BT_VELKEEP, start_state_in_frenet=True)

    # traffic.add_remote_vehicle( 99, 'Ego', [0.0,0.0,0.0, 1.0,0.0,0.0])
    #traffic.add_vehicle( 2, 'Ego', [20.0,0.0,0.0, 0.0,0.0,0.0], BT_VELKEEP)
    #traffic.add_vehicle( 3, 'V3', [0.0,0.0,0.0, 2.0,0.0,0.0], BT_VELKEEP)
    
    
    #GUI / Debug screen
    dashboard = Dashboard(traffic, PLOT_VID)
    
    #SIM EXECUTION START
    print ('SIMULATION START')
    traffic.start()
    dashboard.start()
    while sync_global.tick():
        try:
            #Update Traffic
            traffic.tick(
                sync_global.tick_count, 
                sync_global.delta_time,
                sync_global.sim_time
            )
        except KeyboardInterrupt:
            break
        
    traffic.stop_all()    
    #SIM END
    print('SIMULATION END')
