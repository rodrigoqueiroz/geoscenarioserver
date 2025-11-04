#!/usr/bin/env python3
#rqueiroz@uwaterloo.ca
#d43sharm@uwaterloo.ca
# ---------------------------------------------
# GeoScenario Server
# Starts the Server and controls the traffic simulation loop
# --------------------------------------------

from argparse import ArgumentParser
import os

from geoscenarioserver.GSServerBase import GSServerBase
from geoscenarioserver.mapping.LaneletMap import verify_lanelet_map_file
from geoscenarioserver.dash.Dashboard import wait_for_input, get_screen_parameters
from geoscenarioserver.requirements.RequirementViolationEvents import GlobalTick
from geoscenarioserver.TickSync import TickSync

import logging
log = logging.getLogger("GSServer")

class GSServer(GSServerBase):
    def __init__(self):
        # This way we can reference the base functions directly
        super().__init__()

    def construct_sim_config(self, args):
        """
        Configure SimConfig based on command-line arguments
        Note: sim_config is created by initialize_core_components()
        """
        self.sim_config.show_dashboard = not args.no_dash
        self.sim_config.wait_for_input = args.wait_for_input
        self.sim_config.wait_for_client = args.wait_for_client

    def start(self, args):
        # log.setLevel("INFO")
        log.info('GeoScenario server START')

        # Initialize core components using base class method
        self.initialize_core_components()
        self.construct_sim_config(args)
        btree_locations = self.parse_btree_paths(args.btree_locations)

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
        self.start_traffic()

        #GUI / Debug screen - using base class method
        self.setup_dashboard(args.dash_pos)

        dashboard_interrupted = False
        while sync_global.tick():
            # Check if dashboard is alive using base class method
            if self.sim_config.show_dashboard and not self.is_dashboard_alive():
                dashboard_interrupted = True
                break
            try:
                # Update simulation state tracking
                self.update_simulation_state(sync_global.tick_count, sync_global.delta_time, sync_global.sim_time)

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

        # Shutdown using unified base class method
        self.shutdown(interrupted=dashboard_interrupted)

        #SIM END
        log.info('SIMULATION END')
        log.info('GeoScenario server shutdown')



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
        verify_lanelet_map_file(args.verify_map)
    
    gs = GSServer()
    gs.start(args)

if __name__ == "__main__":
    main()