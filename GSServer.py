#!/usr/bin/env python
#rqueiroz@uwaterloo.ca
#d43sharm@uwaterloo.ca
# ---------------------------------------------
# GeoScenario Server
# Starts the Server and controls the traffic simulation loop
# --------------------------------------------

from argparse import ArgumentParser
from TickSync import TickSync
from SimTraffic import SimTraffic
from SimConfig import SimConfig
from dash.Dashboard import *
from mapping.LaneletMap import *
from ScenarioSetup import *
from lanelet2.projection import UtmProjector
import glog as log

def start_server(args, m=MVelKeepConfig()):
    # log.setLevel("INFO")
    log.info('GeoScenario server START')
    lanelet_map = LaneletMap()
    sim_config = SimConfig()
    traffic = SimTraffic(lanelet_map, sim_config)
    if args.verify_map != "":
        verify_map_file(args.verify_map, lanelet_map)
        return
    
    # SCENARIO SETUP
    if args.gsfile:
        if '.osm' in args.gsfile:
            #GeoScenario XML files (GSParser)
            res = load_geoscenario_from_file(args.gsfile, traffic, sim_config, lanelet_map)
        else:
            #Direct setup
            res = load_geoscenario_from_code(args.gsfile, traffic, sim_config, lanelet_map)
    else: 
        res = load_geoscenario_from_code("", traffic, sim_config, lanelet_map)

    if not res:
        log.error("Failed to load scenario")
        return
       
    
    sync_global = TickSync(rate=sim_config.traffic_rate, realtime=True, block=True, verbose=False, label="EX")
    sync_global.set_timeout(sim_config.timeout)

    #GUI / Debug screen
    dashboard = Dashboard(traffic, sim_config)

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
        #try:
            #Update Traffic
        sim_status = traffic.tick(
            sync_global.tick_count,
            sync_global.delta_time,
            sync_global.sim_time
        )
        if sim_status < 0:
            break
        #except Exception as e:
        #    log.error(e)
        #    break
        
    traffic.stop_all()
    dashboard.quit()
   
    #SIM END
    log.info('SIMULATION END')
    log.info('GeoScenario server shutdown')

def verify_map_file(map_file, lanelet_map:LaneletMap):
    projector = UtmProjector(lanelet2.io.Origin(43.0, -80))
    lanelet_map.load_lanelet_map(map_file, projector)



if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("-s", "--scenario", dest="gsfile", metavar="FILE", default="", help="GeoScenario file. If no file is provided, the GSServer will load a scenario from code")
    parser.add_argument("--verify_map", dest="verify_map", metavar="FILE", default="", help="Lanelet map file")
    parser.add_argument("-q", "--quiet", dest="verbose", default=True, help="don't print messages to stdout")
    parser.add_argument("-n", "--no_dash", dest="no_dash", action="store_true", help="run without the dashboard")
    
    args = parser.parse_args()
    start_server(args)
