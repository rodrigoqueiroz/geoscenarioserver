#!/usr/bin/env python
#rqueiroz@uwaterloo.ca
#d43sharm@uwaterloo.ca
# ---------------------------------------------
# GeoScenario Server
# Controls the problem setup and traffic simulation loop
# --------------------------------------------

from argparse import ArgumentParser
import glog as log

from TickSync import TickSync
from SimTraffic import SimTraffic
from dash.Dashboard import *
from mapping.LaneletMap import *
from lanelet2.projection import UtmProjector
from SimConfig import SimConfig
from gsc.GSParser import GSParser
from sv.ManeuverConfig import *

def start_server(args, m=MVelKeepConfig()):
    # log.setLevel("INFO")

    log.info('GeoScenario server START')
    lanelet_map = LaneletMap()
    sim_config = SimConfig()
    traffic = SimTraffic(lanelet_map, sim_config)

    if args.verify_map != "":
        verify_map_file(args.verify_map, lanelet_map)
        return

    # Scenario SETUP
    if not args.gsfile:
        #Direct setup
        setup_problem(traffic, sim_config, lanelet_map)
    else:
        #Using GeoScenario XML files (GSParser)
        if not setup_problem_from_file(args.gsfile, traffic, sim_config, lanelet_map):
            return

    sync_global = TickSync(rate=sim_config.traffic_rate, realtime=True, block=True, verbose=False, label="EX")
    sync_global.set_timeout(sim_config.timeout)

    #GUI / Debug screen
    dashboard = Dashboard(traffic, sim_config.plot_vid)

    #SIM EXECUTION START
    log.info('SIMULATION START')
    traffic.start()
    show_dashboard = SHOW_DASHBOARD and not args.no_dash
    dashboard.start(show_dashboard)

    if WAIT_FOR_INPUT:
        input("waiting for [ENTER]...")

    while sync_global.tick():
        if show_dashboard and not dashboard._process.is_alive(): # might/might not be wanted
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
    dashboard.quit()
    #SIM END
    log.info('SIMULATION END')
    log.info('GeoScenario server shutdown')

def verify_map_file(map_file, lanelet_map:LaneletMap):
    # TODO: origin needs to be close to the map
    projector = UtmProjector(lanelet2.io.Origin(43.0, -80))
    lanelet_map.load_lanelet_map(map_file, projector)

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
    #traffic.add_remote_vehicle(2, 'Ego', [0.0,0.0,0.0, 1.0,0.0,0.0])

    #traffic.add_remote_vehicle(x, 'Ego', [0.0,0.0,0.0, 1.0,0.0,0.0])

    #adding vehicle at the start of a lanelet
    #traffic.add_vehicle(2, 'V2', [4.0,0.0,0.0, 0.0,0.0,0.0],
    #    sim_config.lanelet_routes[1], 'drive_tree')
    #traffic.add_vehicle(3, 'V3', [8,0.0,0.0, 0.0,0.0,0.0],
    #        sim_config.lanelet_routes[2], 'drive_tree')


def setup_problem_from_file(gsfile, sim_traffic, sim_config, lanelet_map):
    """ Setup scenario from GeoScenario file
    """
    parser = GSParser()
    if not parser.load_and_validate_geoscenario(gsfile):
        log.error("Error loading GeoScenario file")
        return False
    if parser.globalconfig.tags['version'] < 2.0:
        log.error("GSServer requires GeoScenario 2.0 or newer")
        return False

    sim_config.scenario_name = parser.globalconfig.tags['name']
    sim_config.timeout = parser.globalconfig.tags['timeout']
    if 'plotvid' in parser.globalconfig.tags:
        sim_config.plot_vid = parser.globalconfig.tags['plotvid']

    #map
    map_file = 'scenarios/' + parser.globalconfig.tags['lanelet']
    # use origin from gsc file to project nodes to sim frame
    projector = UtmProjector(lanelet2.io.Origin(parser.origin.lat, parser.origin.lon))
    parser.project_nodes(projector)
    lanelet_map.load_lanelet_map(map_file, projector)

    # add remote ego
    if parser.egostart is not None:
        sim_traffic.add_remote_vehicle(99, 'Ego', [0.0,0.0,0.0, 0.0,0.0,0.0])

    # populate traffic and lanelet routes from file
    for vid, vnode in parser.vehicles.items():
        simvid = int(vid)
        btree_root = vnode.tags['btree']
        myroute = vnode.tags['route']

        try:
            # NOTE may not want to prepend vehicle node, in case the user starts the vehicle in the middle of the path
            route_nodes = parser.routes[myroute].nodes
            lanelets_in_route = [ lanelet_map.get_occupying_lanelet(node.x, node.y) for node in route_nodes ]
            sim_config.lanelet_routes[simvid] = lanelet_map.get_route_via(lanelets_in_route)
        except Exception as e:
            log.error("Route generation failed for route {}.".format(myroute))
            raise e

        sim_config.goal_points[simvid] = (route_nodes[-1].x, route_nodes[-1].y)
        sim_traffic.add_vehicle(simvid, vnode.tags['name'], [vnode.x,0.0,0.0, vnode.y,0.0,0.0],
                                sim_config.lanelet_routes[simvid], btree_root)
    return True


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("-s", "--scenario", dest="gsfile", metavar="FILE", default="", help="GeoScenario file")
    parser.add_argument("--verify_map", dest="verify_map", metavar="FILE", default="", help="Lanelet map file")
    parser.add_argument("-q", "--quiet", dest="verbose", default=True, help="don't print messages to stdout")
    parser.add_argument("-n", "--no_dash", dest="no_dash", action="store_true", help="run without the dashboard")
    args = parser.parse_args()
    start_server(args)
