#!/usr/bin/env python3
#rqueiroz@uwaterloo.ca
#d43sharm@uwaterloo.ca
# ---------------------------------------------
# GeoScenario Server
# Starts the Server and controls the traffic simulation loop
# --------------------------------------------

from argparse import ArgumentParser
import os

try:
    from lanelet2.projection import LocalCartesianProjector
    use_local_cartesian=True
except ImportError:
    from lanelet2.projection import UtmProjector
    use_local_cartesian=False

from geoscenarioserver.dash.Dashboard import *
from geoscenarioserver.mapping.LaneletMap import *
from geoscenarioserver.requirements.RequirementViolationEvents import GlobalTick
from geoscenarioserver.ScenarioSetup import *
from geoscenarioserver.SimConfig import *
from geoscenarioserver.SimTraffic import SimTraffic
from geoscenarioserver.TickSync import TickSync
from geoscenarioserver.GSServerBase import GSServerBase

import logging
log = logging.getLogger("GSServer")

class GSServer(GSServerBase):
    def __init__(self):
        # This way we can reference the base functions directly
        super().__init__()

    def start(self, args):
        # log.setLevel("INFO")
        log.info('GeoScenario server START')

        btree_locations = self.parse_btree_paths(args.btree_locations)

        self.sim_config.show_dashboard = not args.no_dash
        self.sim_config.wait_for_input = args.wait_for_input
        self.sim_config.wait_for_client = args.wait_for_client

        # use sim_config after all modifications
        self.traffic = SimTraffic(self.lanelet_map, self.sim_config)

        # SCENARIO SETUP
        if not self.construct_scenario(args.gsfiles, self.traffic, self.sim_config, self.lanelet_map, args.map_path, btree_locations):
            log.error("Failed to load scenario")
            return

        sync_global = TickSync(rate=self.sim_config.traffic_rate, realtime=True, block=True, verbose=False, label="traffic")
        sync_global.set_timeout(self.sim_config.timeout)

        screen_param = get_screen_parameters(args.dash_pos)

        if self.sim_config.wait_for_input:
            wait_for_input(self.sim_config.show_dashboard, args.dash_pos, screen_param)

        #SIM EXECUTION START
        log.info('SIMULATION START')
        self.traffic.start()

        #GUI / Debug screen
        if self.sim_config.show_dashboard:
            dashboard = Dashboard(self.traffic, self.sim_config, screen_param)
            dashboard.start()

        dashboard_interrupted = False
        while sync_global.tick():
            if self.sim_config.show_dashboard and not dashboard._process.is_alive(): # might/might not be wanted
                dashboard_interrupted = True
                break
            try:
                #Update Traffic
                sim_status = self.traffic.tick(
                    sync_global.tick_count,
                    sync_global.delta_time,
                    sync_global.sim_time
                )

                GlobalTick()

                if sim_status < 0:
                    break
            except Exception as e:
                log.error(e)
                break
        sync_global.write_performance_log()
        self.traffic.stop_all(dashboard_interrupted)

        if self.sim_config.show_dashboard and dashboard._process.is_alive():
            dashboard.quit()

        #SIM END
        log.info('SIMULATION END')
        log.info('GeoScenario server shutdown')

def verify_map_file(map_file, lanelet_map:LaneletMap):
    if use_local_cartesian:
        projector = LocalCartesianProjector(lanelet2.io.Origin(43.4681668322, -80.5436763174, 302))
        log.info("Using LocalCartesianProjector")
    else:
        projector = UtmProjector(lanelet2.io.Origin(43.4681668322, -80.5436763174, 302))
        log.info("Using UTMProjector")
    lanelet_map.load_lanelet_map(map_file, projector)

def main():
    """Main entry point for GeoScenario Server"""
    parser = ArgumentParser(description="Starts the GeoScenario Server simulation", allow_abbrev=True)
    parser.add_argument("-s", "--scenario", dest="gsfiles", nargs='*', metavar="FILE", default="", help="GeoScenario file. If no file is provided, the GSServer will load a scenario from code")
    parser.add_argument("--verify_map", dest="verify_map", metavar="FILE", default="", help="Lanelet map file")
    parser.add_argument("-q", "--quiet", dest="verbose", default=True, help="don't print messages to stdout")
    parser.add_argument("-n", "--no-dash", dest="no_dash", action="store_true", help="run without the dashboard")
    parser.add_argument("-m", "--map-path", dest="map_path", default="", help="Set the prefix to append to the value of the attribute `globalconfig->lanelet`")
    parser.add_argument("-b", "--btree-locations", dest="btree_locations", default="", help="Add higher priority locations to search for btrees by agent btypes")
    parser.add_argument("-wi", "--wait-for-input", dest="wait_for_input", action="store_true", help="Wait for the user to press [ENTER] to start the simulation")
    parser.add_argument("-wc", "--wait-for-client", dest="wait_for_client", action="store_true", help="Wait for a valid client state to start the simulation")
    parser.add_argument("-dp", "--dash-pos", default=[], dest="dash_pos", type=float, nargs=4, help="Set the position of the dashboard window (x y width height)")
    parser.add_argument("-d", "--debug", dest="debug", action="store_true", help="Set the logging level to DEBUG instead of INFO")
    parser.add_argument("-fl", "--file-log", dest="file_log", action="store_true", help="Log to $GSS_OUTPUTS/GSServer.log instead of stdout")
    args = parser.parse_args()

    log_level = logging.DEBUG if args.debug else logging.INFO
    if args.file_log:
        filename = os.path.join(
            os.getenv("GSS_OUTPUTS", os.path.join(os.getcwd(), "outputs")),
            "GSServer.log")
        logging.basicConfig(filename=filename, filemode="w", level=log_level)
    else:
        logging.basicConfig(level=log_level)
    
    if args.verify_map != "":
        LaneletMap.verify_map(args.verify_map)
        return

    gs = GSServer()
    gs.start(args)

if __name__ == "__main__":
    main()