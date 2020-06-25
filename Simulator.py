#!/usr/bin/env python
#rqueiroz@gsd.uwaterloo.ca
#d43sharm@uwaterloo.ca
# ---------------------------------------------
# SIMULATOR
# Problem Setup and Simulation Loop
# --------------------------------------------

from TickSync import TickSync
from SimTraffic import *
from dash.DashBoard import *
from util.Constants import *

from Mapping.LaneletMap import *
from lanelet2.projection import UtmProjector
from SimConfig import SimConfig
from gsc.GSParser import GSParser


if __name__ == "__main__":
    sync_global   = TickSync(rate=FRAME_RATE, realtime = True, block=True, verbose=False, label="EX")
    # simulation lasts TIMEOUT seconds
    sync_global.set_timeout(TIMEOUT)

    # load scenario from gsc file
    parser = GSParser()
    parser.load_and_validate_geoscenario("scenarios/example_map_scenario.osm")
    # use origin from gsc file to project nodes to sim frame
    projector = UtmProjector(lanelet2.io.Origin(parser.origin.lat, parser.origin.lon))
    parser.project_nodes(projector)
    lanelet_map = LaneletMap(projector)

    # PROBLEM SETUP
    # Problem setup can be defined directly, or using GeoScenario XML files (GSParser)
    sim_config = SimConfig()
    traffic = SimTraffic()
    # set these BEFORE adding vehicles - also why not using constructor?
    traffic.set_map(lanelet_map)
    traffic.set_sim_config(sim_config)
    
    # setup vehicles and lanelet routes from file
    for vid, vnode in parser.vehicles.items():
        # Use starting point of lanelet as first point in its path
        path_nodes = [vnode] + parser.paths[vnode.tags['path']].nodes
        lanelets_in_path = [ lanelet_map.get_occupying_lanelet(node.x, node.y) for node in path_nodes ]
        sim_id = vnode.tags['simid']

        sim_config.lanelet_routes[sim_id] = lanelet_map.get_route_via(lanelets_in_path)
        traffic.add_vehicle(sim_id, vnode.tags['name'], [vnode.x,0.0,0.0, vnode.y,0.0,0.0],
            sim_config.lanelet_routes[sim_id], BT_VELKEEP)

    #traffic.add_remote_vehicle( 99, 'Ego', [0.0,0.0,0.0, 1.0,0.0,0.0])
    # adding vehicle at the start of a lanelet
    # traffic.add_vehicle(1, 'V1', [4.0,0.0,0.0, 0.0,0.0,0.0],
    #     sim_config.lanelet_routes[1], BT_VELKEEP, start_state_in_frenet=True)
    # test location
    # traffic.add_vehicle( 1, 'V1', [0,0.0,0.0, 0,0.0,0.0], BT_VELKEEP)
    # traffic.add_vehicle(2, 'V2', [8,0.0,0.0, 0.0,0.0,0.0],
    #     sim_config.lanelet_routes[2], BT_VELKEEP, start_state_in_frenet=True)

    # traffic.add_remote_vehicle( 99, 'Ego', [0.0,0.0,0.0, 1.0,0.0,0.0])
    #traffic.add_vehicle( 2, 'Ego', [20.0,0.0,0.0, 0.0,0.0,0.0], BT_VELKEEP)
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
