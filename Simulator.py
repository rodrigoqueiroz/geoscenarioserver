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

    # class for parsing and holding map
    test_map = LaneletMap()
    # load + validate scenario from gsc file
    projector = UtmProjector(lanelet2.io.Origin(49, 8.4))
    parser = GSParser()
    parser.validate_geoscenario("scenarios/example_map_scenario.osm", projector=projector)
    
    # class for holding scenario settings - load from file?
    # sim_config = SimConfig({
        # 1 : test_map.get_route(329661501650965856, 99991), # [99998, 99997, 99996, 99995, 99994, 99993, 99992, 99991],
        # 2 : test_map.get_route(329661501650965856, 99991), # [99998, 99997, 99996, 99995, 99994, 99993, 99992, 99991],
        # })
    # for testing - grab a path to put vehicles on
    # ref_path = test_map.get_global_path_for_route(sim_config.lanelet_routes[1])

    # PROBLEM SETUP
    # Problem setup can be defined directly, or using GeoScenario XML files (GSParser)
    sim_config = SimConfig()
    traffic = SimTraffic()
    # set these BEFORE adding vehicles - also why not using constructor?
    traffic.set_map(test_map)
    traffic.set_sim_config(sim_config)
    
    for vid, vnode in parser.vehicles.items():
        print("found vehicle with id {} start {} and path {}".format(vid, (vnode.x, vnode.y), vnode.tags["path"]))
        # TODO: use path way of vehicle to resolve lanelet route
        # use starting point of lanelet as first point in its path
        path_nodes = [vnode] + parser.paths[vnode.tags['path']].nodes
        lanelets_in_path = [ test_map.get_occupying_lanelet(node.x, node.y) for node in path_nodes ]
        route = test_map.get_route_via(lanelets_in_path)
        start_ll = test_map.get_occupying_lanelet(vnode.x, vnode.y)
        end_ll = test_map.get_occupying_lanelet(path_nodes[-1].x, path_nodes[-1].y)
        sim_config.lanelet_routes[vnode.id] = test_map.get_route(start_ll.id, end_ll.id)
        
        traffic.add_vehicle(vnode.id, vnode.tags['name'], [vnode.x,0.0,0.0, vnode.y,0.0,0.0],
            sim_config.lanelet_routes[vnode.id], BT_VELKEEP)
    exit()
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
