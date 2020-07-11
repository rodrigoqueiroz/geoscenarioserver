#!/usr/bin/env python
#rqueiroz@uwaterloo.ca
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
from lanelet2.projection import UtmProjector
from SimConfig import SimConfig
from gsc.GSParser import GSParser
from argparse import ArgumentParser

def start_server(args):
    print ('GeoScenario server START')
    lanelet_map = LaneletMap()
    sim_config = SimConfig()
    traffic = SimTraffic(lanelet_map, sim_config)

    # Scenario SETUP
    if not args.gsfile:
        #Direct setup
        setup_problem(traffic, sim_config, lanelet_map)
    else:
        #Using GeoScenario XML files (GSParser)
        if not setup_problem_from_file(args.gsfile, traffic, sim_config, lanelet_map):
            return
            
    sync_global   = TickSync(rate=sim_config.traffic_rate, realtime = True, block=True, verbose=False, label="EX")
    sync_global.set_timeout(sim_config.timeout)
    
    
    #GUI / Debug screen
    dashboard = Dashboard(traffic, PLOT_VID)
    
    #SIM EXECUTION START
    print ('SIMULATION START')
    traffic.start()
    dashboard.start()
    while sync_global.tick():
        if not dashboard._process.is_alive(): # might/might not be wanted
            break
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
    print('GeoScenario server shutdown')

def setup_problem(sim_traffic, sim_config, lanelet_map):
    """ Setup scenario directly
    """
    sim_config.scenario_name = "MyScenario"
    sim_config.timeout = 10
    map_file = "scenarios/maps/ll2_round.osm"
    projector = UtmProjector(lanelet2.io.Origin(49.0, 8.4))
    #map
    lanelet_map.load_lanelet_map(map_file, projector)
    #traffic.add_vehicle( 1, 'V1', [ref_path[1].x,0.0,0.0, ref_path[1].y,0.0,0.0],
    #    sim_config.lanelet_routes[1], BT_VELKEEP)
    #traffic.add_remote_vehicle(1, 'Ego', [0.0,0.0,0.0, 1.0,0.0,0.0])
    #adding vehicle at the start of a lanelet
    #traffic.add_vehicle(2, 'V2', [4.0,0.0,0.0, 0.0,0.0,0.0],
    #    sim_config.lanelet_routes[1], 'drive_tree')
    #traffic.add_vehicle(3, 'V3', [8,0.0,0.0, 0.0,0.0,0.0],
    #        sim_config.lanelet_routes[2], 'drive_tree')
    

def setup_problem_from_file(gsfile, sim_traffic, sim_config, lanelet_map):
    """ Setup problem from GeoScenario file
    """
    parser = GSParser()
    if not parser.load_and_validate_geoscenario(gsfile):
        print("Error loading GeoScenario file")
        return False
    if parser.globalconfig.tags['version'] < 2.0:
        print("GSServer requires GeoScenario 2.0 or newer")
        return False
    
    sim_config.scenario_name = parser.globalconfig.tags['name']
    sim_config.timeout = parser.globalconfig.tags['timeout']
    #map
    map_file = 'scenarios/' + parser.globalconfig.tags['lanelet']
    # use origin from gsc file to project nodes to sim frame
    projector = UtmProjector(lanelet2.io.Origin(parser.origin.lat, parser.origin.lon))
    parser.project_nodes(projector)
    lanelet_map.load_lanelet_map(map_file, projector)

    # populate traffic and lanelet routes from file
    for vid, vnode in parser.vehicles.items():
        # Use starting point of lanelet as first point in its path
        path_nodes = [vnode] + parser.paths[vnode.tags['path']].nodes
        lanelets_in_path = [ lanelet_map.get_occupying_lanelet(node.x, node.y) for node in path_nodes ]
        sim_id = vnode.tags['simid']
        btree_root = vnode.tags['btree']
        sim_config.lanelet_routes[sim_id] = lanelet_map.get_route_via(lanelets_in_path)
        sim_config.goal_points[sim_id] = (path_nodes[-1].x, path_nodes[-1].y)
        
        sim_traffic.add_vehicle(sim_id, vnode.tags['name'], [vnode.x,0.0,0.0, vnode.y,0.0,0.0],
            sim_config.lanelet_routes[sim_id], btree_root)
    return True
    
if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("-s", "--scenario", dest="gsfile", metavar="FILE", default="", help="GeoScenario file")
    parser.add_argument("-q", "--quiet", dest="verbose", default=True, help="don't print messages to stdout")
    args = parser.parse_args()
    start_server(args)
